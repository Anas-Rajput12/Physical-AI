# LLM-Based Cognitive Planning for Humanoid Robots

## Introduction to LLM-Based Planning

Large Language Models (LLMs) have emerged as powerful tools for cognitive planning in robotics, enabling robots to understand complex instructions, reason about their environment, and generate appropriate action sequences. For humanoid robots, LLMs provide the cognitive capabilities needed for flexible, human-like task execution and problem-solving.

## LLM Integration Architecture

### Cognitive Planning Pipeline

The LLM-based planning system follows this pipeline:

1. **Command Understanding**: LLM processes natural language commands
2. **Context Analysis**: LLM considers environmental and situational context
3. **Task Decomposition**: LLM breaks down complex tasks into subtasks
4. **Action Sequencing**: LLM generates sequences of robot actions
5. **Execution Monitoring**: LLM monitors execution and adapts plans

### ROS 2 Integration Layer

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
import openai
import json
import asyncio
from typing import Dict, List, Any

class LLMCognitivePlanner(Node):
    def __init__(self):
        super().__init__('llm_cognitive_planner')

        # Subscribers for commands and context
        self.command_sub = self.create_subscription(
            String, '/natural_language_command', self.command_callback, 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # Publishers for plans and status
        self.plan_pub = self.create_publisher(String, '/generated_plan', 10)
        self.status_pub = self.create_publisher(String, '/planning_status', 10)

        # LLM configuration
        self.llm_model = "gpt-4"  # or use open-source alternatives
        self.llm_temperature = 0.3
        self.context_window = []

        # Robot capabilities and environment
        self.robot_capabilities = self.get_robot_capabilities()
        self.environment_context = self.get_environment_context()

    def get_robot_capabilities(self):
        """Define what the robot can do"""
        return {
            "locomotion": ["move_forward", "turn_left", "turn_right", "navigate_to_pose"],
            "manipulation": ["pick_up", "place_down", "grasp", "release"],
            "interaction": ["speak", "listen", "gesture"],
            "sensors": ["camera", "lidar", "imu", "microphone"]
        }

    def get_environment_context(self):
        """Get current environmental context"""
        return {
            "available_objects": ["table", "chair", "cup", "book"],
            "locations": ["kitchen", "living_room", "bedroom"],
            "robot_state": "idle",
            "battery_level": 0.8
        }

    def command_callback(self, msg):
        """Process natural language command with LLM"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Generate plan using LLM
        plan = self.generate_plan_with_llm(command)

        if plan:
            # Publish the plan
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)

            # Execute the plan
            self.execute_plan(plan)

    def generate_plan_with_llm(self, command: str) -> Dict[str, Any]:
        """Generate execution plan using LLM"""
        # Prepare context for LLM
        context = {
            "robot_capabilities": self.robot_capabilities,
            "environment_context": self.environment_context,
            "command": command,
            "instruction": "Generate a step-by-step execution plan for the humanoid robot to complete the given command. Return a JSON object with 'steps' containing an array of actions, each with 'action_type', 'parameters', and 'description'."
        }

        try:
            # Call LLM (using OpenAI API as example)
            response = openai.ChatCompletion.create(
                model=self.llm_model,
                messages=[
                    {"role": "system", "content": "You are a cognitive planner for a humanoid robot. Generate detailed execution plans."},
                    {"role": "user", "content": json.dumps(context, indent=2)}
                ],
                temperature=self.llm_temperature
            )

            plan_text = response.choices[0].message.content

            # Parse the plan
            plan = json.loads(plan_text)
            return plan

        except Exception as e:
            self.get_logger().error(f'LLM planning error: {e}')
            return None
```

## Task Decomposition with LLMs

### Hierarchical Task Planning

```python
class HierarchicalTaskPlanner:
    def __init__(self, llm_client):
        self.llm_client = llm_client

    def decompose_task(self, high_level_task: str, context: Dict) -> List[Dict]:
        """Decompose high-level task into executable subtasks"""
        decomposition_prompt = f"""
        Decompose the following high-level task into specific, executable subtasks for a humanoid robot:

        Task: {high_level_task}

        Context:
        - Robot capabilities: {context.get('robot_capabilities', {})}
        - Environment: {context.get('environment_context', {})}
        - Constraints: {context.get('constraints', {})}

        Provide the decomposition as a JSON list of subtasks, where each subtask has:
        - 'name': descriptive name
        - 'description': what the subtask accomplishes
        - 'required_capabilities': list of robot capabilities needed
        - 'estimated_duration': in seconds
        - 'dependencies': list of subtask names that must be completed first
        - 'success_criteria': how to determine if the subtask was successful

        Example format:
        [
            {{
                "name": "navigate_to_kitchen",
                "description": "Move robot to the kitchen area",
                "required_capabilities": ["navigation", "locomotion"],
                "estimated_duration": 30,
                "dependencies": [],
                "success_criteria": "Robot is in kitchen area as determined by localization"
            }}
        ]
        """

        try:
            response = self.llm_client.generate(decomposition_prompt)
            subtasks = json.loads(response)
            return subtasks
        except Exception as e:
            print(f"Task decomposition error: {e}")
            return []

    def create_execution_plan(self, subtasks: List[Dict]) -> List[Dict]:
        """Convert subtasks into executable robot commands"""
        execution_plan = []

        for subtask in subtasks:
            # Map subtask to specific robot actions
            actions = self.map_subtask_to_actions(subtask)
            execution_plan.extend(actions)

        return execution_plan

    def map_subtask_to_actions(self, subtask: Dict) -> List[Dict]:
        """Map subtask to specific ROS 2 actions"""
        # This would map high-level subtasks to specific ROS 2 messages/services
        action_mapping = {
            "navigation": {"type": "NavigateToPose", "service": "navigate_to_pose"},
            "manipulation": {"type": "FollowJointTrajectory", "service": "arm_controller"},
            "perception": {"type": "Image", "topic": "camera/image_raw"},
            "interaction": {"type": "String", "topic": "tts/text"}
        }

        # Return appropriate action structure
        return [{"action": subtask, "mapping": action_mapping}]
```

## Context-Aware Planning

### Environmental Context Integration

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np

class ContextAwarePlanner(Node):
    def __init__(self):
        super().__init__('context_aware_planner')

        # Subscribers for environmental context
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/humanoid/lidar', self.lidar_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        # Context storage
        self.bridge = CvBridge()
        self.current_image = None
        self.current_lidar = None
        self.current_map = None
        self.environment_objects = []

    def image_callback(self, msg):
        """Process camera image for object recognition"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Perform object detection (simplified)
            objects = self.detect_objects(cv_image)

            # Update environment context
            self.environment_objects = objects

        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def detect_objects(self, image):
        """Detect objects in image (simplified implementation)"""
        # In practice, use YOLO, Detectron2, or similar
        # For this example, we'll return a mock detection
        return [
            {"name": "cup", "position": [1.0, 2.0, 0.0], "confidence": 0.9},
            {"name": "book", "position": [0.5, 1.5, 0.0], "confidence": 0.85}
        ]

    def lidar_callback(self, msg):
        """Process LiDAR data for obstacle detection"""
        # Process LiDAR data to identify obstacles and free space
        self.current_lidar = msg

    def map_callback(self, msg):
        """Process occupancy grid for navigation context"""
        self.current_map = msg

    def get_environment_context(self):
        """Get current environmental context for LLM"""
        context = {
            "detected_objects": self.environment_objects,
            "obstacle_locations": self.extract_obstacle_locations(),
            "navigable_areas": self.extract_navigable_areas(),
            "current_pose": self.get_robot_pose()
        }
        return context

    def extract_obstacle_locations(self):
        """Extract obstacle locations from sensor data"""
        if self.current_lidar:
            # Process LiDAR data to identify obstacle positions
            obstacles = []
            angle_min = self.current_lidar.angle_min
            angle_increment = self.current_lidar.angle_increment

            for i, range_val in enumerate(self.current_lidar.ranges):
                if range_val < self.current_lidar.range_max and range_val > self.current_lidar.range_min:
                    angle = angle_min + i * angle_increment
                    x = range_val * np.cos(angle)
                    y = range_val * np.sin(angle)
                    obstacles.append([x, y, range_val])

            return obstacles
        return []

    def extract_navigable_areas(self):
        """Extract navigable areas from map"""
        if self.current_map:
            # Process occupancy grid to identify free space
            # This is a simplified example
            width = self.current_map.info.width
            height = self.current_map.info.height
            resolution = self.current_map.info.resolution

            free_cells = []
            for i, cell_value in enumerate(self.current_map.data):
                if cell_value == 0:  # Free space
                    x = (i % width) * resolution
                    y = (i // width) * resolution
                    free_cells.append([x, y])

            return free_cells
        return []
```

## LLM-Based Action Selection

### Cognitive Action Selection

```python
class CognitiveActionSelector:
    def __init__(self, llm_client):
        self.llm_client = llm_client
        self.action_history = []

    def select_action(self, observation: Dict, goal: str) -> Dict:
        """Select next action based on observation and goal using LLM"""
        context = {
            "current_observation": observation,
            "goal": goal,
            "previous_actions": self.action_history[-5:],  # Last 5 actions
            "robot_capabilities": self.get_robot_capabilities(),
            "environment_context": observation.get("environment", {}),
            "instruction": "Based on the current observation and goal, select the most appropriate action for the humanoid robot. Return a JSON object with 'action_type', 'parameters', 'reasoning', and 'expected_outcome'."
        }

        try:
            response = self.llm_client.generate(json.dumps(context, indent=2))
            action = json.loads(response)
            self.action_history.append(action)
            return action
        except Exception as e:
            print(f"Action selection error: {e}")
            return self.get_default_action()

    def get_robot_capabilities(self):
        """Return robot capabilities for action selection"""
        return {
            "navigation": {
                "types": ["move_to", "follow_path", "explore"],
                "constraints": {"max_speed": 0.5, "min_turn_radius": 0.3}
            },
            "manipulation": {
                "types": ["pick", "place", "grasp", "release"],
                "constraints": {"max_payload": 2.0, "reach_distance": 1.0}
            },
            "interaction": {
                "types": ["speak", "listen", "gesture"],
                "constraints": {"max_speech_length": 30}
            }
        }

    def get_default_action(self):
        """Return default action if LLM fails"""
        return {
            "action_type": "wait",
            "parameters": {},
            "reasoning": "Default action due to planning failure",
            "expected_outcome": "Robot waits for further instructions"
        }

    def validate_action(self, action: Dict, observation: Dict) -> bool:
        """Validate if action is feasible in current context"""
        action_type = action.get("action_type")
        params = action.get("parameters", {})

        # Check if robot has required capabilities
        if not self.check_robot_capabilities(action_type):
            return False

        # Check environmental constraints
        if not self.check_environmental_constraints(action_type, params, observation):
            return False

        return True

    def check_robot_capabilities(self, action_type: str) -> bool:
        """Check if robot can perform the action"""
        # Implementation would check robot's current state and capabilities
        return True

    def check_environmental_constraints(self, action_type: str, params: Dict, observation: Dict) -> bool:
        """Check if action is feasible in current environment"""
        # Implementation would check environmental constraints
        return True
```

## Planning with Uncertainty and Adaptation

### Uncertainty-Aware Planning

```python
class UncertaintyAwarePlanner:
    def __init__(self, llm_client):
        self.llm_client = llm_client
        self.uncertainty_models = {}

    def plan_with_uncertainty(self, goal: str, initial_context: Dict) -> Dict:
        """Generate plan that accounts for uncertainty"""
        uncertainty_prompt = f"""
        Generate a robust plan for: {goal}

        Context: {json.dumps(initial_context, indent=2)}

        Consider the following sources of uncertainty:
        - Sensor noise and limited field of view
        - Dynamic environment with moving objects
        - Potential robot actuation errors
        - Communication delays

        Provide a plan that includes:
        1. Primary action sequence
        2. Alternative actions for each step
        3. Conditions that trigger plan switching
        4. Recovery strategies for common failures
        5. Confidence estimates for each action

        Return as JSON with structure:
        {{
            "primary_plan": [...],
            "alternative_plans": [...],
            "switching_conditions": [...],
            "recovery_strategies": [...],
            "confidence_estimates": [...]
        }}
        """

        try:
            response = self.llm_client.generate(uncertainty_prompt)
            plan = json.loads(response)
            return plan
        except Exception as e:
            print(f"Uncertainty-aware planning error: {e}")
            return self.get_default_robust_plan()

    def get_default_robust_plan(self):
        """Return default robust plan"""
        return {
            "primary_plan": [],
            "alternative_plans": [],
            "switching_conditions": [],
            "recovery_strategies": [
                {"condition": "navigation_failure", "action": "return_to_known_location"},
                {"condition": "object_not_found", "action": "explore_search_pattern"},
                {"condition": "grasp_failure", "action": "retry_with_different_approach"}
            ],
            "confidence_estimates": []
        }

    def adapt_plan(self, current_plan: Dict, new_observation: Dict) -> Dict:
        """Adapt plan based on new observations"""
        adaptation_prompt = f"""
        Current plan: {json.dumps(current_plan, indent=2)}
        New observation: {json.dumps(new_observation, indent=2)}

        Adapt the plan based on the new observation. Consider:
        - Has the goal changed?
        - Are there new obstacles or opportunities?
        - Did previous actions succeed as expected?
        - Should we switch to an alternative plan?

        Return the adapted plan in the same format as the original.
        """

        try:
            response = self.llm_client.generate(adaptation_prompt)
            adapted_plan = json.loads(response)
            return adapted_plan
        except Exception as e:
            print(f"Plan adaptation error: {e}")
            return current_plan  # Return original plan if adaptation fails
```

## Integration with ROS 2 Actions and Services

### LLM-ROS 2 Bridge

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

class LLMROS2Bridge(Node):
    def __init__(self):
        super().__init__('llm_ros2_bridge')

        # Action clients for robot capabilities
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.arm_client = ActionClient(self, FollowJointTrajectory, 'arm_controller')

        # Publishers for other capabilities
        self.tts_pub = self.create_publisher(String, '/tts/input', 10)
        self.gesture_pub = self.create_publisher(String, '/gesture/command', 10)

        # Plan execution state
        self.current_plan = None
        self.plan_index = 0
        self.execution_active = False

    def execute_plan(self, plan: Dict):
        """Execute plan generated by LLM"""
        self.current_plan = plan
        self.plan_index = 0
        self.execution_active = True

        self.get_logger().info(f'Executing plan with {len(plan.get("steps", []))} steps')

        # Execute first step
        if plan.get('steps'):
            self.execute_next_step()

    def execute_next_step(self):
        """Execute the next step in the plan"""
        if not self.current_plan or not self.execution_active:
            return

        steps = self.current_plan.get('steps', [])
        if self.plan_index >= len(steps):
            # Plan completed
            self.get_logger().info('Plan execution completed')
            self.execution_active = False
            return

        current_step = steps[self.plan_index]
        self.get_logger().info(f'Executing step: {current_step.get("action_type")}')

        # Execute based on action type
        action_type = current_step.get('action_type')
        parameters = current_step.get('parameters', {})

        if action_type == 'navigate_to_pose':
            self.execute_navigation_action(parameters)
        elif action_type == 'speak':
            self.execute_speech_action(parameters)
        elif action_type == 'gesture':
            self.execute_gesture_action(parameters)
        else:
            self.get_logger().warn(f'Unknown action type: {action_type}')
            self.plan_index += 1
            self.execute_next_step()

    def execute_navigation_action(self, params: Dict):
        """Execute navigation action"""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation action server not available')
            return

        goal_msg = NavigateToPose.Goal()

        # Set target pose from parameters
        pose_data = params.get('pose', {})
        goal_msg.pose.header.frame_id = pose_data.get('frame_id', 'map')
        goal_msg.pose.pose.position.x = pose_data.get('x', 0.0)
        goal_msg.pose.pose.position.y = pose_data.get('y', 0.0)
        goal_msg.pose.pose.position.z = pose_data.get('z', 0.0)

        # Set orientation (assuming quaternion format)
        orientation = pose_data.get('orientation', {})
        goal_msg.pose.pose.orientation.x = orientation.get('x', 0.0)
        goal_msg.pose.pose.orientation.y = orientation.get('y', 0.0)
        goal_msg.pose.pose.orientation.z = orientation.get('z', 0.0)
        goal_msg.pose.pose.orientation.w = orientation.get('w', 1.0)

        # Send navigation goal
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_done_callback)

    def navigation_done_callback(self, future):
        """Handle navigation completion"""
        goal_handle = future.result()
        if goal_handle.status == 2:  # SUCCEEDED
            self.get_logger().info('Navigation completed successfully')
            self.plan_index += 1
            self.execute_next_step()
        else:
            self.get_logger().warn('Navigation failed, continuing to next step')
            self.plan_index += 1
            self.execute_next_step()

    def execute_speech_action(self, params: Dict):
        """Execute speech action"""
        text = params.get('text', '')
        if text:
            tts_msg = String()
            tts_msg.data = text
            self.tts_pub.publish(tts_msg)

        # Move to next step after a delay
        self.create_timer(2.0, self.move_to_next_step)

    def execute_gesture_action(self, params: Dict):
        """Execute gesture action"""
        gesture_type = params.get('gesture', 'idle')
        gesture_msg = String()
        gesture_msg.data = gesture_type
        self.gesture_pub.publish(gesture_msg)

        # Move to next step after gesture duration
        self.create_timer(1.0, self.move_to_next_step)

    def move_to_next_step(self):
        """Move to the next step in the plan"""
        if self.current_plan:
            self.plan_index += 1
            self.execute_next_step()
```

## Best Practices for LLM-Based Planning

### 1. Safety and Validation
- Always validate LLM-generated plans before execution
- Implement safety checks and emergency stops
- Use formal methods to verify critical plans

### 2. Performance Optimization
- Cache frequently used plans and patterns
- Use efficient prompting techniques
- Implement plan execution monitoring

### 3. Context Management
- Maintain coherent context across planning sessions
- Update context based on execution results
- Handle context switching for multiple tasks

### 4. Human-in-the-Loop
- Provide mechanisms for human oversight
- Allow plan interruption and modification
- Implement explanation capabilities

## Troubleshooting LLM-Based Planning

### Common Issues and Solutions

1. **Plan Feasibility**: LLMs may generate plans that are not feasible for the robot
   - Solution: Implement validation layers and constraint checking

2. **Context Window Limitations**: LLMs have limited memory
   - Solution: Use external memory systems and summarization

3. **Execution Failures**: Plans may fail during execution
   - Solution: Implement robust error handling and recovery

4. **Safety Concerns**: LLMs may generate unsafe actions
   - Solution: Implement safety constraints and human oversight

## Exercises

1. Implement an LLM-based planner that can decompose complex household tasks
2. Create a context-aware planning system that uses sensor data
3. Develop a plan adaptation system that handles execution failures
4. Build a safety validation layer for LLM-generated robot actions
5. Implement human-robot collaboration with shared planning authority