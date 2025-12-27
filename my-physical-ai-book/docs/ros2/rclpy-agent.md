# Python AI Agent Integration with rclpy

## Introduction to rclpy

rclpy is the Python client library for ROS 2, providing a Python API to interact with the ROS 2 ecosystem. For AI agents in humanoid robotics, rclpy serves as the bridge between high-level AI decision-making algorithms and the low-level robotic control systems.

## Setting Up rclpy for AI Agents

### Installation and Basic Setup

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import tensorflow as tf  # or your preferred AI framework
```

### Creating an AI Agent Node

```python
class HumanoidAIAgent(Node):
    def __init__(self):
        super().__init__('humanoid_ai_agent')

        # QoS profile for reliable communication
        qos_profile = QoSProfile(depth=10)

        # Publishers for commanding the robot
        self.cmd_publisher = self.create_publisher(
            Twist, 'cmd_vel', qos_profile)
        self.action_publisher = self.create_publisher(
            String, 'robot_actions', qos_profile)

        # Subscribers for sensor data
        self.sensor_subscriber = self.create_subscription(
            SensorMsg, 'sensor_data', self.sensor_callback, qos_profile)
        self.camera_subscriber = self.create_subscription(
            Image, 'camera/image_raw', self.camera_callback, qos_profile)

        # Initialize AI model
        self.ai_model = self.initialize_ai_model()

        # Timer for AI decision loop
        self.ai_timer = self.create_timer(0.1, self.ai_decision_loop)

        self.get_logger().info('Humanoid AI Agent initialized')

    def initialize_ai_model(self):
        # Load your trained AI model here
        # This could be a neural network, decision tree, etc.
        model = tf.keras.models.load_model('humanoid_behavior_model')
        return model

    def sensor_callback(self, msg):
        # Process sensor data
        self.latest_sensor_data = msg

    def camera_callback(self, msg):
        # Process camera data
        self.latest_camera_data = msg

    def ai_decision_loop(self):
        # Main AI decision-making loop
        if hasattr(self, 'latest_sensor_data') and hasattr(self, 'latest_camera_data'):
            # Prepare input for AI model
            ai_input = self.prepare_ai_input(
                self.latest_sensor_data,
                self.latest_camera_data
            )

            # Get AI decision
            action = self.ai_model.predict(ai_input)

            # Execute action
            self.execute_action(action)

    def prepare_ai_input(self, sensor_data, camera_data):
        # Convert ROS messages to format suitable for AI model
        # This is a simplified example
        sensor_array = np.array([
            sensor_data.linear_acceleration.x,
            sensor_data.linear_acceleration.y,
            sensor_data.linear_acceleration.z,
            sensor_data.angular_velocity.x,
            sensor_data.angular_velocity.y,
            sensor_data.angular_velocity.z
        ])

        # Process camera data (simplified)
        camera_processed = self.process_camera_data(camera_data)

        # Combine sensor and camera data
        ai_input = np.concatenate([sensor_array, camera_processed])
        return ai_input.reshape(1, -1)  # Reshape for model prediction

    def process_camera_data(self, camera_msg):
        # Convert ROS Image message to numpy array
        # This is a simplified example - in practice, you'd use cv2
        # Convert from ROS image message to OpenCV format
        # Process the image for AI input
        pass

    def execute_action(self, action_prediction):
        # Convert AI prediction to robot command
        cmd_vel = Twist()

        # Example: if action is to move forward
        if action_prediction[0] > 0.5:
            cmd_vel.linear.x = 0.5  # Move forward
        else:
            cmd_vel.linear.x = 0.0

        # Publish command
        self.cmd_publisher.publish(cmd_vel)
```

## Advanced AI Agent Patterns

### State Machine Integration

```python
from enum import Enum

class RobotState(Enum):
    IDLE = 1
    WALKING = 2
    MANIPULATING = 3
    BALANCING = 4
    EMERGENCY_STOP = 5

class StatefulAIAgent(Node):
    def __init__(self):
        super().__init__('stateful_ai_agent')
        self.current_state = RobotState.IDLE
        self.previous_state = None

        # Setup publishers, subscribers, and timers
        self.state_publisher = self.create_publisher(String, 'robot_state', 10)
        self.ai_timer = self.create_timer(0.05, self.state_machine_loop)

    def state_machine_loop(self):
        # Process sensor data to determine if state should change
        new_state = self.determine_state()

        if new_state != self.current_state:
            self.handle_state_transition(self.current_state, new_state)
            self.current_state = new_state

        # Execute behavior for current state
        self.execute_current_state_behavior()

        # Publish current state
        state_msg = String()
        state_msg.data = self.current_state.name
        self.state_publisher.publish(state_msg)

    def determine_state(self):
        # Use sensor data and AI model to determine appropriate state
        # This could involve neural networks, rule-based systems, etc.
        pass

    def handle_state_transition(self, old_state, new_state):
        # Clean up old state, initialize new state
        self.get_logger().info(f'State transition: {old_state.name} -> {new_state.name}')
        # Add any transition-specific logic here

    def execute_current_state_behavior(self):
        # Execute the behavior for the current state
        if self.current_state == RobotState.WALKING:
            self.execute_walking_behavior()
        elif self.current_state == RobotState.BALANCING:
            self.execute_balancing_behavior()
        # etc.
```

### Multi-Agent Coordination

```python
class CoordinatedAIAgent(Node):
    def __init__(self):
        super().__init__('coordinated_ai_agent')

        # Publisher for coordination messages
        self.coordination_publisher = self.create_publisher(
            String, 'coordination_msgs', 10)

        # Subscriber for coordination messages from other agents
        self.coordination_subscriber = self.create_subscription(
            String, 'coordination_msgs', self.coordination_callback, 10)

        # Keep track of other agents
        self.other_agents = {}

    def coordination_callback(self, msg):
        # Process coordination message from other agents
        coordination_data = self.parse_coordination_message(msg.data)

        # Update internal state based on coordination data
        self.update_coordination_state(coordination_data)

    def parse_coordination_message(self, message_str):
        # Parse coordination message (could be JSON, custom format, etc.)
        import json
        try:
            return json.loads(message_str)
        except:
            return None

    def update_coordination_state(self, coordination_data):
        # Update coordination state based on received data
        if coordination_data and 'agent_id' in coordination_data:
            agent_id = coordination_data['agent_id']
            self.other_agents[agent_id] = coordination_data
```

## Integration with Machine Learning Frameworks

### TensorFlow/Keras Integration

```python
import tensorflow as tf
from tensorflow import keras

class TFAgent(Node):
    def __init__(self):
        super().__init__('tf_agent')

        # Load pre-trained model
        self.model = self.load_model()

        # Setup for real-time inference
        self.inference_timer = self.create_timer(0.03, self.run_inference)

    def load_model(self):
        # Load model from file or create new one
        try:
            model = keras.models.load_model('humanoid_behavior_model.h5')
            self.get_logger().info('Model loaded successfully')
        except:
            # Create and compile a new model if loading fails
            model = self.create_default_model()
            self.get_logger().info('Created default model')
        return model

    def run_inference(self):
        # Get latest sensor data
        if hasattr(self, 'latest_sensor_data'):
            # Prepare input
            input_data = self.prepare_input(self.latest_sensor_data)

            # Run inference
            with self.model.as_default():
                prediction = self.model.predict(input_data, verbose=0)

            # Process prediction
            self.process_prediction(prediction)

    def prepare_input(self, sensor_data):
        # Prepare sensor data for model input
        # Convert ROS message to numpy array suitable for model
        pass

    def process_prediction(self, prediction):
        # Convert model output to robot actions
        pass
```

### PyTorch Integration

```python
import torch
import torch.nn as nn

class PyTorchAgent(Node):
    def __init__(self):
        super().__init__('pytorch_agent')

        # Load PyTorch model
        self.model = self.load_pytorch_model()
        self.model.eval()  # Set to evaluation mode

    def load_pytorch_model(self):
        # Load trained PyTorch model
        model = YourPyTorchModel()  # Define your model class
        try:
            model.load_state_dict(torch.load('humanoid_model.pth'))
            self.get_logger().info('PyTorch model loaded successfully')
        except:
            self.get_logger().info('Could not load model, using untrained model')
        return model

    def run_pytorch_inference(self, input_data):
        # Convert input to tensor
        input_tensor = torch.tensor(input_data, dtype=torch.float32)

        # Run inference
        with torch.no_grad():
            output = self.model(input_tensor)

        # Convert output back to numpy for ROS usage
        return output.numpy()
```

## Best Practices for AI Agent Integration

1. **Threading Considerations**: Keep AI inference separate from ROS communication threads
2. **Memory Management**: Be mindful of memory usage when running AI models on robot hardware
3. **Real-time Performance**: Optimize AI models for real-time execution requirements
4. **Model Updates**: Implement mechanisms for updating AI models during operation
5. **Error Handling**: Handle AI model failures gracefully without stopping robot operation
6. **Safety Integration**: Ensure AI decisions respect safety constraints and emergency stops

## Exercises

1. Implement a simple AI agent that responds to voice commands using ROS 2
2. Create a neural network that processes sensor data to determine robot state
3. Build a multi-agent system where multiple AI nodes coordinate behavior
4. Integrate a reinforcement learning model with ROS 2 for adaptive behavior