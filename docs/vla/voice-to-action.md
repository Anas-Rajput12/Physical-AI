# Voice-to-Action: Converting Speech Commands to Robot Actions

## Introduction to Voice-to-Action Systems

Voice-to-Action systems bridge human natural language and robot behavior, enabling intuitive human-robot interaction. For humanoid robots, voice command processing is particularly important as it supports the natural communication expectations humans have with human-like agents.

## Voice Command Processing Pipeline

The typical voice-to-action pipeline consists of:

1. **Audio Capture**: Collecting voice commands from users
2. **Speech Recognition**: Converting speech to text
3. **Natural Language Understanding**: Interpreting the meaning of commands
4. **Action Mapping**: Translating commands to robot actions
5. **Action Execution**: Executing mapped actions on the robot

## Audio Capture and Preprocessing

### Microphone Array Setup

For humanoid robots, audio capture requires consideration of the robot's form factor and intended use:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
import pyaudio
import numpy as np

class AudioCaptureNode(Node):
    def __init__(self):
        super().__init__('audio_capture')

        # Publisher for audio data
        self.audio_pub = self.create_publisher(AudioData, '/audio_input', 10)

        # Audio parameters
        self.rate = 16000  # Sample rate
        self.chunk = 1024  # Buffer size
        self.channels = 1  # Mono
        self.format = pyaudio.paInt16  # 16-bit audio

        # Initialize audio stream
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        # Timer for continuous audio capture
        self.timer = self.create_timer(0.1, self.capture_audio)

    def capture_audio(self):
        """Capture audio from microphone and publish"""
        try:
            data = self.stream.read(self.chunk, exception_on_overflow=False)

            # Convert to AudioData message
            audio_msg = AudioData()
            audio_msg.data = data
            audio_msg.encoding = 'PCM_16'
            audio_msg.sample_rate = self.rate
            audio_msg.channels = self.channels

            self.audio_pub.publish(audio_msg)
        except Exception as e:
            self.get_logger().error(f'Audio capture error: {e}')
```

### Audio Preprocessing

For humanoid robots operating in noisy environments, audio preprocessing is crucial:

```python
import numpy as np
from scipy import signal
import webrtcvad

class AudioPreprocessor:
    def __init__(self, sample_rate=16000):
        self.sample_rate = sample_rate
        self.vad = webrtcvad.Vad(2)  # Aggressiveness mode 2

        # Noise reduction parameters
        self.noise_threshold = 0.01
        self.speech_buffer_size = 1600  # 100ms at 16kHz

    def preprocess_audio(self, audio_data):
        """Preprocess audio for speech recognition"""
        # Convert bytes to numpy array
        audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

        # Apply noise reduction
        audio_filtered = self.apply_noise_reduction(audio_array)

        # Voice activity detection
        is_speech = self.detect_voice_activity(audio_filtered)

        if is_speech:
            return audio_filtered
        else:
            return None

    def apply_noise_reduction(self, audio_array):
        """Apply basic noise reduction"""
        # Simple spectral subtraction for noise reduction
        # In practice, use more sophisticated techniques like Wiener filtering
        return audio_array

    def detect_voice_activity(self, audio_array):
        """Detect voice activity using WebRTC VAD"""
        # WebRTC VAD expects 10, 20, or 30ms frames
        frame_size = int(0.02 * self.sample_rate)  # 20ms frame

        for i in range(0, len(audio_array), frame_size):
            frame = audio_array[i:i+frame_size]
            if len(frame) < frame_size:
                continue

            # Convert to bytes for WebRTC VAD
            frame_bytes = (frame * 32768).astype(np.int16).tobytes()

            # Check for voice activity
            if self.vad.is_speech(frame_bytes, self.sample_rate):
                return True

        return False
```

## Speech Recognition with Whisper

OpenAI's Whisper model provides excellent speech recognition capabilities for voice-to-action systems:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
import torch
import whisper
import numpy as np
import io
from scipy.io import wavfile

class WhisperSpeechRecognizer(Node):
    def __init__(self):
        super().__init__('whisper_speech_recognizer')

        # Subscribe to audio input
        self.audio_sub = self.create_subscription(
            AudioData, '/audio_input', self.audio_callback, 10)

        # Publisher for recognized text
        self.text_pub = self.create_publisher(String, '/recognized_text', 10)

        # Load Whisper model
        self.get_logger().info('Loading Whisper model...')
        self.model = whisper.load_model("base")  # Use "small" or "medium" for better accuracy

        # Audio preprocessing
        self.preprocessor = AudioPreprocessor()

        # Store audio chunks for processing
        self.audio_buffer = []
        self.buffer_size = 16000  # 1 second of audio at 16kHz

    def audio_callback(self, msg):
        """Process incoming audio data"""
        # Preprocess audio
        processed_audio = self.preprocessor.preprocess_audio(msg.data)

        if processed_audio is not None:
            # Add to buffer
            self.audio_buffer.extend(processed_audio)

            # If buffer is large enough, process
            if len(self.audio_buffer) >= self.buffer_size:
                self.process_audio_buffer()

    def process_audio_buffer(self):
        """Process accumulated audio buffer with Whisper"""
        if len(self.audio_buffer) == 0:
            return

        # Convert buffer to audio array
        audio_array = np.array(self.audio_buffer, dtype=np.float32)

        # Pad or truncate to minimum length if needed
        if len(audio_array) < 16000:  # Minimum 1 second
            padding = 16000 - len(audio_array)
            audio_array = np.pad(audio_array, (0, padding), 'constant')

        try:
            # Transcribe with Whisper
            result = self.model.transcribe(
                audio_array,
                language='en',
                task='transcribe'
            )

            text = result['text'].strip()

            if text:  # Only publish non-empty text
                text_msg = String()
                text_msg.data = text
                self.text_pub.publish(text_msg)
                self.get_logger().info(f'Recognized: {text}')

        except Exception as e:
            self.get_logger().error(f'Whisper transcription error: {e}')
        finally:
            # Clear buffer after processing
            self.audio_buffer = []
```

## Natural Language Understanding for Commands

### Command Parser

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import spacy
import re

class CommandParser(Node):
    def __init__(self):
        super().__init__('command_parser')

        # Subscribe to recognized text
        self.text_sub = self.create_subscription(
            String, '/recognized_text', self.text_callback, 10)

        # Publishers for parsed commands
        self.command_pub = self.create_publisher(String, '/parsed_command', 10)
        self.navigation_pub = self.create_publisher(Pose, '/navigation_goal', 10)

        # Load spaCy model for NLP
        try:
            self.nlp = spacy.load("en_core_web_sm")
        except OSError:
            self.get_logger().warn("spaCy model not found, using basic parsing")
            self.nlp = None

        # Define command patterns
        self.command_patterns = {
            'navigation': [
                r'go to (.+)',
                r'move to (.+)',
                r'walk to (.+)',
                r'go to the (.+)',
                r'navigate to (.+)'
            ],
            'manipulation': [
                r'pick up (.+)',
                r'grab (.+)',
                r'take (.+)',
                r'get (.+)',
                r'lift (.+)'
            ],
            'greeting': [
                r'hello',
                r'hi',
                r'hey'
            ]
        }

    def text_callback(self, msg):
        """Process recognized text and extract commands"""
        text = msg.data.lower().strip()

        # Parse the command
        command_type, parsed_command = self.parse_command(text)

        if command_type and parsed_command:
            # Create command message
            cmd_msg = String()
            cmd_msg.data = f"{command_type}:{parsed_command}"
            self.command_pub.publish(cmd_msg)

            # Handle specific command types
            if command_type == 'navigation':
                self.handle_navigation_command(parsed_command)
            elif command_type == 'manipulation':
                self.handle_manipulation_command(parsed_command)

    def parse_command(self, text):
        """Parse command using pattern matching and NLP"""
        for cmd_type, patterns in self.command_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text)
                if match:
                    # Extract the object/location from the command
                    extracted = match.group(1).strip()
                    return cmd_type, extracted

        # If no pattern matched, return unknown
        return 'unknown', text

    def handle_navigation_command(self, location):
        """Handle navigation commands by mapping to coordinates"""
        # This would map location names to coordinates
        # In a real system, this would use a map or semantic localization
        location_map = {
            'kitchen': {'x': 5.0, 'y': 2.0},
            'living room': {'x': 0.0, 'y': 0.0},
            'bedroom': {'x': -3.0, 'y': 4.0},
            'bathroom': {'x': -1.0, 'y': -2.0}
        }

        if location in location_map:
            goal_pose = Pose()
            goal_pose.position.x = location_map[location]['x']
            goal_pose.position.y = location_map[location]['y']
            goal_pose.position.z = 0.0
            goal_pose.orientation.w = 1.0  # No rotation

            self.navigation_pub.publish(goal_pose)
```

## Action Mapping and Execution

### Voice Command to Robot Action Mapping

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
import time

class VoiceActionMapper(Node):
    def __init__(self):
        super().__init__('voice_action_mapper')

        # Subscribe to parsed commands
        self.command_sub = self.create_subscription(
            String, '/parsed_command', self.command_callback, 10)

        # Robot control publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.action_status_pub = self.create_publisher(Bool, '/action_status', 10)

        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Store command mappings
        self.command_actions = {
            'navigation': self.execute_navigation,
            'greeting': self.execute_greeting,
            'stop': self.execute_stop
        }

    def command_callback(self, msg):
        """Execute action based on parsed command"""
        try:
            cmd_type, cmd_data = msg.data.split(':', 1)

            if cmd_type in self.command_actions:
                self.command_actions[cmd_type](cmd_data)
            else:
                self.get_logger().warn(f'Unknown command type: {cmd_type}')

        except ValueError:
            self.get_logger().error(f'Invalid command format: {msg.data}')

    def execute_navigation(self, location):
        """Execute navigation to specified location"""
        self.get_logger().info(f'Navigating to {location}')

        # For this example, we'll assume location is mapped to coordinates
        # In practice, you'd have a more sophisticated location mapping system
        location_coords = self.map_location_to_coordinates(location)

        if location_coords:
            self.navigate_to_pose(location_coords)
        else:
            self.get_logger().warn(f'Unknown location: {location}')

    def map_location_to_coordinates(self, location):
        """Map location name to coordinates"""
        # This would typically query a semantic map
        location_map = {
            'kitchen': (5.0, 2.0, 0.0),  # x, y, theta
            'living room': (0.0, 0.0, 0.0),
            'bedroom': (-3.0, 4.0, 0.0),
            'bathroom': (-1.0, -2.0, 0.0)
        }

        if location in location_map:
            return location_map[location]
        else:
            # Try to find closest match
            # This is a simple example - real implementation would use NLP
            for key in location_map:
                if location in key or key in location:
                    return location_map[key]

        return None

    def navigate_to_pose(self, coords):
        """Navigate to specific pose using Nav2"""
        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = coords[0]
        goal_msg.pose.pose.position.y = coords[1]
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(0, 0, coords[2])
        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]

        # Wait for action server and send goal
        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_done_callback)

    def navigation_done_callback(self, future):
        """Handle navigation completion"""
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Navigation goal accepted')
        else:
            self.get_logger().info('Navigation goal rejected')

    def execute_greeting(self, greeting_data):
        """Execute greeting behavior"""
        self.get_logger().info(f'Responding to greeting: {greeting_data}')

        # Example: Move arms in greeting motion
        # This would typically use action servers for complex behaviors
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def execute_stop(self, stop_data):
        """Execute stop behavior"""
        self.get_logger().info('Stopping robot movement')

        # Stop all movement
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
```

## Advanced Voice Processing Techniques

### Context-Aware Command Processing

```python
class ContextAwareVoiceProcessor(Node):
    def __init__(self):
        super().__init__('context_aware_voice_processor')

        # Subscribers
        self.text_sub = self.create_subscription(
            String, '/recognized_text', self.text_callback, 10)
        self.robot_state_sub = self.create_subscription(
            String, '/robot_state', self.state_callback, 10)

        # Context storage
        self.current_context = {
            'location': 'unknown',
            'last_action': 'none',
            'task_in_progress': False,
            'human_interaction': False
        }

        # Command history
        self.command_history = []

    def text_callback(self, msg):
        """Process text with context awareness"""
        text = msg.data.lower().strip()

        # Analyze context before processing command
        enhanced_command = self.enhance_command_with_context(text)

        # Process enhanced command
        self.process_enhanced_command(enhanced_command)

    def enhance_command_with_context(self, text):
        """Enhance command with current context"""
        # Example: "there" should refer to current location context
        if 'there' in text and self.current_context['location'] != 'unknown':
            text = text.replace('there', f"to {self.current_context['location']}")

        # Example: "it" should refer to last mentioned object
        if 'it' in text and self.current_context['last_action'] != 'none':
            # This would need more sophisticated tracking
            pass

        return text

    def state_callback(self, msg):
        """Update context based on robot state"""
        self.current_context['robot_state'] = msg.data

    def process_enhanced_command(self, enhanced_command):
        """Process command with enhanced context"""
        # Parse and execute with context
        pass
```

## Voice Command Validation and Error Handling

### Command Validation System

```python
class VoiceCommandValidator(Node):
    def __init__(self):
        super().__init__('voice_command_validator')

        # Subscribers and publishers
        self.unvalidated_sub = self.create_subscription(
            String, '/raw_command', self.validate_command, 10)
        self.validated_pub = self.create_publisher(
            String, '/validated_command', 10)
        self.feedback_pub = self.create_publisher(
            String, '/command_feedback', 10)

        # Command validation rules
        self.validation_rules = {
            'navigation': self.validate_navigation,
            'manipulation': self.validate_manipulation,
            'interaction': self.validate_interaction
        }

    def validate_command(self, msg):
        """Validate voice command before execution"""
        command_type, command_data = self.parse_command_type(msg.data)

        if command_type in self.validation_rules:
            is_valid, reason = self.validation_rules[command_type](command_data)

            if is_valid:
                # Command is valid, publish for execution
                validated_msg = String()
                validated_msg.data = f"{command_type}:{command_data}"
                self.validated_pub.publish(validated_msg)
            else:
                # Command is invalid, provide feedback
                feedback_msg = String()
                feedback_msg.data = f"Command invalid: {reason}"
                self.feedback_pub.publish(feedback_msg)
        else:
            feedback_msg = String()
            feedback_msg.data = f"Unknown command type: {command_type}"
            self.feedback_pub.publish(feedback_msg)

    def validate_navigation(self, location):
        """Validate navigation commands"""
        # Check if location is in known map
        known_locations = ['kitchen', 'living room', 'bedroom', 'bathroom']

        if location in known_locations:
            return True, "Location valid"
        else:
            # Try to find similar location
            for known_loc in known_locations:
                if self.string_similarity(location, known_loc) > 0.7:
                    return True, f"Assuming location: {known_loc}"

            return False, f"Unknown location: {location}"

    def string_similarity(self, str1, str2):
        """Calculate string similarity using basic algorithm"""
        from difflib import SequenceMatcher
        return SequenceMatcher(None, str1, str2).ratio()
```

## Best Practices for Voice-to-Action Systems

### 1. Robustness
- Implement fallback mechanisms for recognition failures
- Use confidence scores to determine when to ask for clarification
- Maintain graceful degradation when components fail

### 2. Privacy
- Process voice data locally when possible
- Implement data encryption for sensitive commands
- Provide clear privacy controls

### 3. Responsiveness
- Optimize audio processing pipelines for real-time performance
- Use streaming recognition for faster response times
- Implement command queuing for busy periods

### 4. User Experience
- Provide clear audio feedback for recognized commands
- Implement natural language responses
- Support command interruption and cancellation

## Exercises

1. Implement a complete voice-to-action pipeline using Whisper for speech recognition
2. Create a context-aware command parser for a humanoid robot
3. Develop a validation system that checks commands against environmental constraints
4. Build a feedback system that confirms robot understanding of voice commands
5. Implement voice command interruption and correction mechanisms