# ROS 2 Communication: Nodes, Topics, and Services

## Nodes: The Building Blocks of ROS 2

Nodes are the fundamental computational units in ROS 2. Each node is an independent process that performs a specific function within the robotic system. In humanoid robotics, nodes might handle:

- Sensor data processing (IMU, cameras, LiDAR)
- Motor control and actuation
- Perception and object recognition
- Path planning and navigation
- High-level decision making
- Human-robot interaction

### Creating a Node

Here's a basic example of a ROS 2 node in Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HumanoidSensorNode(Node):
    def __init__(self):
        super().__init__('humanoid_sensor_node')
        self.publisher_ = self.create_publisher(String, 'sensor_data', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Sensor reading: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    humanoid_sensor_node = HumanoidSensorNode()
    rclpy.spin(humanoid_sensor_node)
    humanoid_sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics: Asynchronous Communication

Topics enable asynchronous, many-to-many communication where publishers send messages to named topics and subscribers receive messages from those topics. This pattern is ideal for:

- Sensor data streams (camera images, IMU readings)
- Robot state information (joint positions, battery levels)
- Environmental observations (detected objects, map updates)

### Topic Communication Pattern

```python
# Publisher example
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

    def publish_joint_states(self, positions):
        msg = JointState()
        msg.name = ['hip_joint', 'knee_joint', 'ankle_joint']
        msg.position = positions
        self.publisher_.publish(msg)
```

```python
# Subscriber example
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointMonitor(Node):
    def __init__(self):
        super().__init__('joint_monitor')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Joint positions: {msg.position}')
```

## Services: Synchronous Request-Response

Services provide synchronous, request-response communication where a client sends a request and waits for a response. This pattern is useful for:

- Configuration changes (setting robot parameters)
- Calibration procedures
- Action execution (move to position, grasp object)
- Query operations (get robot status, current pose)

### Service Communication Pattern

First, define the service interface (e.g., `SetJointPosition.srv`):
```
# Request
float64[] positions
---
# Response
bool success
string message
```

Then implement the service server:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class JointService(Node):
    def __init__(self):
        super().__init__('joint_service')
        self.srv = self.create_service(
            SetBool,  # Replace with your custom service type
            'set_joint_position',
            self.set_joint_position_callback)

    def set_joint_position_callback(self, request, response):
        # Process the request and set joint positions
        self.get_logger().info(f'Setting joint positions: {request.data}')
        response.success = True
        response.message = 'Joint positions set successfully'
        return response
```

And the client:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool  # Replace with your service type

class JointClient(Node):
    def __init__(self):
        super().__init__('joint_client')
        self.cli = self.create_client(SetBool, 'set_joint_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, positions):
        request = SetBool.Request()  # Replace with your request type
        request.data = positions
        self.future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Humanoid-Specific Communication Patterns

Humanoid robots have unique communication requirements due to their complexity:

### Sensor Integration
- Multiple IMU sensors for balance
- Stereo cameras for depth perception
- Force/torque sensors in joints
- Pressure sensors in feet

### Control Coordination
- Centralized vs. distributed control
- Real-time vs. non-real-time components
- Safety and emergency stop systems

### Example: Balance Control Communication

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')
        self.imu_subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def imu_callback(self, msg):
        # Process IMU data for balance
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration

        # Calculate corrective actions
        cmd_vel = Twist()
        # Implement balance control logic here
        self.cmd_vel_publisher.publish(cmd_vel)
```

## Best Practices for Humanoid ROS 2 Systems

1. **Message Rate Management**: Balance between responsiveness and computational load
2. **Real-time Considerations**: Separate real-time critical tasks from non-critical ones
3. **Error Handling**: Implement robust error handling for sensor failures
4. **Safety Systems**: Design emergency stop and safety communication channels
5. **Resource Management**: Monitor CPU and memory usage across all nodes
6. **Logging Strategy**: Implement comprehensive logging for debugging complex behaviors

## Exercises

1. Create a node that publishes joint state information for a simple humanoid model
2. Implement a subscriber that monitors sensor data and logs anomalies
3. Design a service that allows external control of humanoid walking parameters
4. Build a communication bridge between different sensor modalities