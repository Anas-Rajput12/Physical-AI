# Nav2: Path Planning for Humanoid Robots

## Introduction to Nav2

Navigation2 (Nav2) is the state-of-the-art navigation framework for ROS 2, designed to provide robust, reliable, and efficient path planning and navigation capabilities. For humanoid robots, Nav2 requires special considerations due to their complex kinematics, balance requirements, and bipedal locomotion patterns.

## Nav2 Architecture Overview

Nav2 follows a behavior tree-based architecture that provides:

- **Modularity**: Independent components that can be customized
- **Flexibility**: Configurable behaviors for different robot types
- **Robustness**: Built-in recovery behaviors and safety mechanisms
- **Scalability**: Support for various sensor configurations and environments

### Core Components

- **Navigation Server**: Main orchestrator that manages navigation tasks
- **Planner Server**: Handles global and local path planning
- **Controller Server**: Manages robot motion control
- **Recovery Server**: Provides recovery behaviors for navigation failures
- **Lifecycle Manager**: Manages component lifecycles

## Nav2 for Humanoid Robotics

### Humanoid-Specific Challenges

Humanoid robots present unique navigation challenges:

- **Bipedal Locomotion**: Requires careful footstep planning and balance control
- **Dynamic Balance**: Must maintain stability during movement
- **Complex Kinematics**: Many degrees of freedom affect navigation
- **Human-like Movement**: Should move in a natural, human-like manner
- **Social Navigation**: Need to navigate safely around humans

### Nav2 Adaptations for Humanoids

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
import math

class HumanoidNav2Client(Node):
    def __init__(self):
        super().__init__('humanoid_nav2_client')

        # Create action client for navigation
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

    def navigate_to_pose(self, x, y, theta):
        """Send navigation goal to Nav2 with humanoid-specific parameters"""
        goal_msg = NavigateToPose.Goal()

        # Set target pose
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0  # Humanoid height

        # Convert angle to quaternion for orientation
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(0, 0, theta)
        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]

        # Set humanoid-specific navigation parameters
        goal_msg.behavior_tree_id = "humanoid_navigation_bt"  # Custom BT for humanoids
        goal_msg.navigation_goal.pose = goal_msg.pose

        # Wait for action server
        self.nav_to_pose_client.wait_for_server()

        # Send navigation goal
        future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback with humanoid-specific metrics"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Navigation progress: {feedback.current_pose}')

    def goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
```

## Global Path Planning for Humanoids

### Humanoid-Aware Global Planner

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from action_msgs.msg import GoalStatus

class HumanoidGlobalPlanner(Node):
    def __init__(self):
        super().__init__('humanoid_global_planner')

        # Action server for path computation
        self._action_server = ActionServer(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
            self.execute_callback)

        # Costmap for humanoid navigation
        self.costmap = self.initialize_humanoid_costmap()

    def initialize_humanoid_costmap(self):
        """Initialize costmap with humanoid-specific parameters"""
        # Create costmap considering humanoid dimensions and capabilities
        # Include factors like:
        # - Minimum turning radius for bipedal movement
        # - Step height limitations
        # - Balance constraints
        # - Social navigation preferences
        pass

    def execute_callback(self, goal_handle):
        """Execute path planning with humanoid constraints"""
        goal = goal_handle.request.goal
        start = goal_handle.request.start
        planner_id = goal_handle.request.planner_id

        self.get_logger().info(f'Computing path for humanoid robot')

        # Plan path considering humanoid kinematics
        path = self.plan_humanoid_path(start, goal)

        if path is not None:
            result = ComputePathToPose.Result()
            result.path = path
            goal_handle.succeed()
            return result
        else:
            goal_handle.abort()
            result = ComputePathToPose.Result()
            result.path = Path()
            return result

    def plan_humanoid_path(self, start, goal):
        """Plan path with humanoid-specific constraints"""
        # Implement path planning algorithm (A*, Dijkstra, etc.)
        # Consider humanoid-specific constraints:
        # - Maintain balance during path execution
        # - Plan for step locations and foot placement
        # - Avoid areas that require complex maneuvers
        # - Consider energy efficiency for walking

        # Example: Modified A* with humanoid constraints
        path = self.humanoid_astar(start, goal)
        return path

    def humanoid_astar(self, start, goal):
        """A* algorithm adapted for humanoid navigation"""
        # Implementation considering humanoid kinematics
        # Include balance constraints, step planning, etc.
        pass
```

## Local Path Planning and Control

### Humanoid Local Planner

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Path
from geometry_msgs.msg import Point

class HumanoidLocalPlanner(Node):
    def __init__(self):
        super().__init__('humanoid_local_planner')

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/humanoid/lidar', self.laser_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/global_plan', self.path_callback, 10)

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Robot state
        self.current_pose = None
        self.current_twist = None
        self.global_path = None
        self.path_index = 0

        # Humanoid-specific parameters
        self.step_size = 0.3  # Maximum step size for humanoid
        self.turn_radius = 0.5  # Minimum turning radius
        self.balance_threshold = 0.1  # Balance maintenance threshold

    def odom_callback(self, msg):
        """Update robot pose and velocity"""
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist

    def laser_callback(self, msg):
        """Process laser data for local obstacle avoidance"""
        # Check for obstacles in path
        obstacles = self.detect_obstacles(msg)

        # Plan local trajectory considering obstacles
        local_cmd = self.plan_local_trajectory(obstacles)

        # Publish command
        self.cmd_pub.publish(local_cmd)

    def path_callback(self, msg):
        """Update global path"""
        self.global_path = msg.poses
        self.path_index = 0

    def detect_obstacles(self, laser_msg):
        """Detect obstacles from laser scan with humanoid considerations"""
        obstacles = []
        angle_min = laser_msg.angle_min
        angle_increment = laser_msg.angle_increment

        for i, range_val in enumerate(laser_msg.ranges):
            if range_val < laser_msg.range_max and range_val > laser_msg.range_min:
                angle = angle_min + i * angle_increment
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)

                # Consider humanoid width for obstacle detection
                if abs(y) < 0.5:  # Humanoid width consideration
                    obstacles.append(Point(x=x, y=y, z=0.0))

        return obstacles

    def plan_local_trajectory(self, obstacles):
        """Plan local trajectory with humanoid balance constraints"""
        cmd = Twist()

        if self.global_path is None or self.path_index >= len(self.global_path):
            return cmd

        # Get next waypoint
        target_pose = self.global_path[self.path_index].pose.position

        # Calculate distance to target
        dist_to_target = math.sqrt(
            (target_pose.x - self.current_pose.position.x)**2 +
            (target_pose.y - self.current_pose.position.y)**2)

        # If close to target, move to next waypoint
        if dist_to_target < 0.3:  # Waypoint tolerance
            self.path_index += 1
            if self.path_index >= len(self.global_path):
                # Reached goal
                return cmd

        # Calculate direction to target
        target_angle = math.atan2(
            target_pose.y - self.current_pose.position.y,
            target_pose.x - self.current_pose.position.x)

        # Current robot angle
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)

        # Calculate angle difference
        angle_diff = self.normalize_angle(target_angle - current_yaw)

        # Humanoid-specific control logic
        if abs(angle_diff) > 0.2:  # Need to turn
            cmd.angular.z = self.calculate_angular_velocity(angle_diff)
        else:
            # Move forward with humanoid step constraints
            cmd.linear.x = min(0.3, self.calculate_linear_velocity(obstacles))

        return cmd

    def calculate_angular_velocity(self, angle_diff):
        """Calculate safe angular velocity for humanoid balance"""
        # Limit angular velocity to maintain balance
        max_angular_vel = 0.5  # rad/s
        k = 0.5  # Proportional gain
        angular_vel = k * angle_diff

        return max(-max_angular_vel, min(max_angular_vel, angular_vel))

    def calculate_linear_velocity(self, obstacles):
        """Calculate safe linear velocity considering obstacles"""
        # Check if path is clear ahead
        if not self.is_path_clear(obstacles):
            return 0.0

        # Humanoid walking speed limits
        max_linear_vel = 0.4  # m/s for safe humanoid walking
        return min(max_linear_vel, 0.3)  # Conservative speed

    def is_path_clear(self, obstacles):
        """Check if path ahead is clear for humanoid"""
        # Check for obstacles in front of robot
        for obs in obstacles:
            # Simple check: if obstacle is within humanoid's width and close range
            if (obs.x > 0 and obs.x < 1.0 and abs(obs.y) < 0.4):
                return False
        return True

    def get_yaw_from_quaternion(self, quat):
        """Extract yaw angle from quaternion"""
        import tf_transformations
        euler = tf_transformations.euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w])
        return euler[2]

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
```

## Behavior Trees for Humanoid Navigation

### Custom Behavior Tree for Humanoid Navigation

```xml
<!-- Custom behavior tree for humanoid navigation -->
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <PipelineSequence name="NavigateWithRecovery">
            <ReactiveSequence name="Navigate">
                <IsPathValid/>
                <ComputePathToPose goal="{goal}" path="{path}" planner_id="humanoid_planner"/>
                <FollowPath path="{path}" controller_id="humanoid_controller"/>
            </ReactiveSequence>
            <ReactiveFallback name="RecoveryFallback">
                <RecoveryNode name="ClearingRotation" number_of_recoveries_attribute="number_of_recoveries_used">
                    <SimpleCondition script="return recovery_enabled_"/>
                    <Spin spin_dist="1.57"/>
                </RecoveryNode>
                <RecoveryNode name="HumanoidWait" number_of_recoveries_attribute="number_of_recoveries_used">
                    <SimpleCondition script="return recovery_enabled_"/>
                    <Wait wait_duration="5"/>
                </RecoveryNode>
            </ReactiveFallback>
        </PipelineSequence>
    </BehaviorTree>
</root>
```

## Social Navigation for Humanoids

### Humanoid Social Navigation

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from people_msgs.msg import People
from std_msgs.msg import Bool

class HumanoidSocialNavigation(Node):
    def __init__(self):
        super().__init__('humanoid_social_navigation')

        # Subscribe to detected people
        self.people_sub = self.create_subscription(
            People, '/people', self.people_callback, 10)

        # Subscribe to navigation state
        self.nav_state_sub = self.create_subscription(
            String, '/navigation/state', self.nav_state_callback, 10)

        # Publisher for social navigation adjustments
        self.social_cmd_pub = self.create_publisher(Twist, '/social_cmd_vel', 10)

        # Store detected people
        self.people = []
        self.nav_active = False

    def people_callback(self, msg):
        """Process detected people for social navigation"""
        self.people = msg.people

        if self.nav_active:
            # Adjust navigation based on people presence
            self.adjust_navigation_for_social_context()

    def nav_state_callback(self, msg):
        """Update navigation state"""
        self.nav_active = (msg.data == 'ACTIVE')

    def adjust_navigation_for_social_context(self):
        """Adjust navigation based on social context"""
        if not self.people:
            return

        # Calculate social forces based on detected people
        social_force = self.calculate_social_force()

        # Modify navigation commands based on social context
        self.apply_social_modifications(social_force)

    def calculate_social_force(self):
        """Calculate social forces from detected people"""
        social_force = Point(x=0.0, y=0.0, z=0.0)

        for person in self.people:
            # Calculate distance to person
            dist = math.sqrt(
                (person.position.x - 0)**2 + (person.position.y - 0)**2)

            # Calculate repulsive force if too close
            if dist < 1.5:  # Social distance threshold
                # Force away from person
                force_mag = max(0, 1.5 - dist)  # Stronger force when closer
                force_angle = math.atan2(
                    person.position.y, person.position.x)

                # Add repulsive force
                social_force.x -= force_mag * math.cos(force_angle)
                social_force.y -= force_mag * math.sin(force_angle)

        return social_force

    def apply_social_modifications(self, social_force):
        """Apply social modifications to navigation"""
        # This would modify the current navigation command
        # based on social forces calculated
        cmd = Twist()
        cmd.linear.x = 0.0  # This would be modified based on social context
        cmd.angular.z = 0.0  # This would be modified based on social context

        self.social_cmd_pub.publish(cmd)
```

## Configuration for Humanoid Navigation

### Nav2 Configuration File for Humanoids

```yaml
# Navigation configuration for humanoid robot
bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Use custom behavior tree for humanoid navigation
    default_nav_to_pose_bt_xml: "humanoid_navigation_tree.xml"
    # Use custom plugins for humanoid navigation
    plugin_lib_names: ["humanoid_navigate_to_pose_action"]

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    # Use humanoid-specific controller
    progress_checker_plugin: "humanoid_progress_checker"
    goal_checker_plugin: "humanoid_goal_checker"
    controller_plugins: ["HumanoidMpcController"]

    # Humanoid MPC Controller parameters
    HumanoidMpcController:
      plugin: "nav2_mpc_controller::MpcController"
      # Humanoid-specific parameters
      prediction_horizon: 1.0
      control_horizon: 5
      robot_model:
        # Humanoid robot model parameters
        base_frame_id: "base_link"
        state_space:
          nx: 6  # State dimension for humanoid
          nu: 2  # Control dimension for humanoid
        control_bounds:
          vx_max: 0.4
          vy_max: 0.2
          wz_max: 0.5

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: false
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      # Humanoid-specific costmap settings
      footprint: "[ [0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2] ]"
      footprint_padding: 0.01
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /humanoid/lidar
          max_obstacle_height: 2.0  # Humanoid height consideration
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: True
        inflation_radius: 0.5  # Humanoid safety margin
        cost_scaling_factor: 3.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: false
      # Humanoid-specific global costmap
      footprint: "[ [0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2] ]"
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /humanoid/lidar
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 10.0
          raytrace_min_range: 0.0
          obstacle_max_range: 8.0
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: True
        inflation_radius: 0.7  # Larger for humanoid safety
        cost_scaling_factor: 2.0

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    # Use custom humanoid path planner
    planner_plugins: ["HumanoidGridBasedPlanner"]
    HumanoidGridBasedPlanner:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

## Best Practices for Humanoid Navigation

### 1. Balance Considerations
- Always maintain dynamic balance during navigation
- Plan paths that allow for stable foot placement
- Use appropriate walking gaits for different terrains

### 2. Safety First
- Implement multiple safety layers
- Ensure emergency stop functionality
- Use appropriate safety margins in costmaps

### 3. Energy Efficiency
- Optimize walking patterns for energy efficiency
- Consider terrain difficulty in path planning
- Use appropriate speeds for different scenarios

### 4. Human-Like Navigation
- Plan paths that humans would naturally take
- Consider social navigation norms
- Implement smooth, natural movements

## Troubleshooting Nav2 for Humanoids

### Common Issues

1. **Path Following Problems**:
   - Verify humanoid kinematic constraints
   - Check controller parameters
   - Ensure proper odometry

2. **Balance Issues During Navigation**:
   - Implement balance feedback control
   - Use appropriate walking patterns
   - Consider center of mass during turns

3. **Performance Problems**:
   - Optimize costmap resolution
   - Use appropriate planning frequencies
   - Implement efficient algorithms

## Exercises

1. Configure Nav2 for a simulated humanoid robot in Gazebo
2. Implement a custom humanoid-aware local planner
3. Create a behavior tree for humanoid social navigation
4. Tune navigation parameters for stable humanoid locomotion
5. Test navigation performance in various environments with obstacles