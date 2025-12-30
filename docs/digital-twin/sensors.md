# Robot Sensors: LiDAR, Depth Camera, IMU

## Overview of Robot Sensors

Sensors are the eyes, ears, and sensory organs of humanoid robots, providing critical information about the robot's state and its environment. In humanoid robotics, multiple sensor modalities work together to enable perception, navigation, balance, and interaction. This chapter covers the three most important sensor types for humanoid robots: LiDAR for 3D mapping, depth cameras for visual perception, and IMUs for balance and orientation.

## Inertial Measurement Unit (IMU)

### What is an IMU?

An Inertial Measurement Unit (IMU) combines multiple sensors to measure the robot's specific force, angular rate, and sometimes the magnetic field surrounding the robot. For humanoid robots, IMUs are critical for:

- Balance and posture control
- Motion tracking and navigation
- Orientation estimation
- Fall detection and recovery

### IMU Components

A typical IMU includes:

- **Accelerometer**: Measures linear acceleration in three axes (X, Y, Z)
- **Gyroscope**: Measures angular velocity around three axes
- **Magnetometer**: Measures magnetic field strength and direction (provides compass functionality)

### IMU in Humanoid Robotics

For humanoid robots, IMUs are typically placed:
- In the torso/chest area (main body orientation)
- In each foot (contact detection and balance)
- In the head (gaze direction and neck movement)

### IMU Data Processing

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import numpy as np
from scipy.spatial.transform import Rotation as R

class ImuProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')

        # IMU data subscriber
        self.imu_sub = self.create_subscription(
            Imu, '/humanoid/imu', self.imu_callback, 10)

        # Balance control publisher
        self.balance_pub = self.create_publisher(
            Float64, '/balance_correction', 10)

        # Store previous values for filtering
        self.previous_orientation = None
        self.orientation_history = []

    def imu_callback(self, msg):
        # Extract IMU data
        linear_acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        angular_vel = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        # Process orientation
        orientation = self.process_orientation(msg.orientation)

        # Calculate balance metrics
        balance_metrics = self.calculate_balance_metrics(
            linear_acc, angular_vel, orientation)

        # Publish balance correction if needed
        if balance_metrics['tilt_angle'] > 0.1:  # 0.1 radians threshold
            correction_msg = Float64()
            correction_msg.data = balance_metrics['correction']
            self.balance_pub.publish(correction_msg)

    def process_orientation(self, orientation_quat):
        # Convert quaternion to Euler angles
        quat = [orientation_quat.x, orientation_quat.y,
                orientation_quat.z, orientation_quat.w]
        r = R.from_quat(quat)
        euler = r.as_euler('xyz', degrees=False)

        return {
            'roll': euler[0],
            'pitch': euler[1],
            'yaw': euler[2]
        }

    def calculate_balance_metrics(self, linear_acc, angular_vel, orientation):
        # Calculate tilt angle from orientation
        tilt_angle = np.sqrt(orientation['roll']**2 + orientation['pitch']**2)

        # Calculate balance correction based on tilt
        correction = -0.5 * orientation['pitch']  # Simple proportional control

        return {
            'tilt_angle': tilt_angle,
            'correction': correction,
            'angular_velocity_magnitude': np.linalg.norm(angular_vel)
        }
```

## LiDAR Sensors

### LiDAR Fundamentals

Light Detection and Ranging (LiDAR) sensors emit laser pulses and measure the time it takes for them to return after reflecting off objects. For humanoid robots, LiDAR provides:

- 3D mapping of the environment
- Obstacle detection and avoidance
- Navigation and path planning
- Localization (SLAM)

### LiDAR Types for Humanoid Robots

- **2D LiDAR**: Single plane scanning (cost-effective, good for ground-level obstacles)
- **3D LiDAR**: Multi-plane scanning (comprehensive environment mapping)
- **Solid-state LiDAR**: No moving parts, more robust (emerging technology)

### LiDAR Data Processing

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import numpy as np

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        # LiDAR subscriber
        self.lidar_sub = self.create_subscription(
            LaserScan, '/humanoid/lidar', self.lidar_callback, 10)

        # Obstacle detection publisher
        self.obstacle_pub = self.create_publisher(
            Marker, '/obstacles', 10)

        # Navigation safety publisher
        self.safety_pub = self.create_publisher(
            Bool, '/navigation_safe', 10)

    def lidar_callback(self, msg):
        # Convert polar coordinates to Cartesian
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        valid_ranges = []
        valid_angles = []

        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max:
                valid_ranges.append(r)
                valid_angles.append(angles[i])

        # Detect obstacles
        obstacles = self.detect_obstacles(valid_ranges, valid_angles, msg.angle_min)

        # Check navigation safety
        safe_to_navigate = self.check_navigation_safety(obstacles)

        # Publish results
        self.publish_obstacles(obstacles)

        safety_msg = Bool()
        safety_msg.data = safe_to_navigate
        self.safety_pub.publish(safety_msg)

    def detect_obstacles(self, ranges, angles, angle_min):
        obstacles = []

        for i, r in enumerate(ranges):
            if r < 1.0:  # Obstacle within 1 meter
                x = r * np.cos(angles[i])
                y = r * np.sin(angles[i])

                # Check if obstacle is at walking height for humanoid
                if -0.8 < y < 0.8:  # Adjust based on robot dimensions
                    obstacles.append({'x': x, 'y': y, 'distance': r})

        return obstacles

    def check_navigation_safety(self, obstacles):
        # Check if path is clear in front of robot
        front_obstacles = [obs for obs in obstacles if obs['x'] > 0 and abs(obs['y']) < 0.5]
        return len(front_obstacles) == 0

    def publish_obstacles(self, obstacles):
        marker = Marker()
        marker.header.frame_id = "laser_frame"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "obstacles"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        # Set marker properties
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0

        for obs in obstacles:
            point = Point()
            point.x = obs['x']
            point.y = obs['y']
            point.z = 0.0
            marker.points.append(point)

        self.obstacle_pub.publish(marker)
```

## Depth Cameras

### Depth Camera Technology

Depth cameras provide both color and depth information, making them invaluable for humanoid robots. Common technologies include:

- **Stereo Vision**: Two cameras to calculate depth from parallax
- **Structured Light**: Projects patterns to calculate depth
- **Time-of-Flight**: Measures light travel time for depth

### Applications in Humanoid Robotics

- **Object Recognition**: Identify and classify objects in the environment
- **Human Detection**: Detect and track humans for interaction
- **Grasping**: Provide depth information for manipulation tasks
- **Navigation**: Dense 3D mapping for precise navigation

### Depth Camera Processing

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Point

class DepthCameraProcessor(Node):
    def __init__(self):
        super().__init__('depth_camera_processor')

        self.bridge = CvBridge()

        # Subscribers for color and depth images
        self.color_sub = self.create_subscription(
            Image, '/humanoid/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/humanoid/camera/depth/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/humanoid/camera/color/camera_info', self.info_callback, 10)

        # Store camera parameters
        self.camera_matrix = None
        self.distortion_coeffs = None

        # Processed data
        self.latest_color = None
        self.latest_depth = None

    def info_callback(self, msg):
        # Extract camera intrinsic parameters
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)

    def color_callback(self, msg):
        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting color image: {e}')

    def depth_callback(self, msg):
        try:
            # Depth image is typically in 16-bit format (mm)
            depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            self.latest_depth = depth_image.astype(np.float32) / 1000.0  # Convert to meters

            # Process depth data if we have both color and depth
            if self.latest_color is not None:
                self.process_depth_data()
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')

    def process_depth_data(self):
        # Find objects in the scene
        objects = self.detect_objects(self.latest_color, self.latest_depth)

        # Calculate 3D positions of objects
        for obj in objects:
            if self.camera_matrix is not None:
                # Calculate 3D position from 2D image coordinates and depth
                center_x, center_y = obj['center']
                depth_value = self.latest_depth[center_y, center_x]

                if depth_value > 0:  # Valid depth
                    # Convert pixel coordinates to 3D world coordinates
                    world_pos = self.pixel_to_world(
                        center_x, center_y, depth_value, self.camera_matrix)

                    obj['world_position'] = world_pos

    def detect_objects(self, color_image, depth_image):
        # Simple object detection using color thresholding
        # In practice, you'd use more sophisticated methods like YOLO or other DNNs

        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Example: Detect red objects
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask1 + mask2

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        objects = []
        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Filter small contours
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2

                objects.append({
                    'center': (center_x, center_y),
                    'bbox': (x, y, w, h),
                    'area': cv2.contourArea(contour)
                })

        return objects

    def pixel_to_world(self, u, v, depth, camera_matrix):
        # Convert pixel coordinates to world coordinates
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]

        # Convert to camera coordinates
        x_cam = (u - cx) * depth / fx
        y_cam = (v - cy) * depth / fy
        z_cam = depth

        # For a humanoid robot, we might want to transform to robot base frame
        # This would involve additional TF transforms
        return Point(x=x_cam, y=y_cam, z=z_cam)
```

## Sensor Fusion for Humanoid Robots

### Combining Multiple Sensors

Humanoid robots benefit from sensor fusion, combining data from multiple sensors to improve perception and decision-making:

```python
class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        # All sensor subscribers
        self.imu_sub = self.create_subscription(Imu, '/humanoid/imu', self.imu_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/humanoid/lidar', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/humanoid/camera/color/image_raw', self.camera_callback, 10)

        # Fusion result publisher
        self.fusion_pub = self.create_publisher(String, '/sensor_fusion_result', 10)

        # Time synchronization
        self.latest_imu = None
        self.latest_lidar = None
        self.latest_camera = None

    def imu_callback(self, msg):
        self.latest_imu = (msg, self.get_clock().now())
        self.fuse_sensors_if_ready()

    def lidar_callback(self, msg):
        self.latest_lidar = (msg, self.get_clock().now())
        self.fuse_sensors_if_ready()

    def camera_callback(self, msg):
        self.latest_camera = (msg, self.get_clock().now())
        self.fuse_sensors_if_ready()

    def fuse_sensors_if_ready(self):
        # Check if all sensors have recent data
        if all([self.latest_imu, self.latest_lidar, self.latest_camera]):
            # Check timestamp synchronization
            imu_time, lidar_time, camera_time = [s[1] for s in
                [self.latest_imu, self.latest_lidar, self.latest_camera]]

            time_diff = max(imu_time, lidar_time, camera_time) - min(imu_time, lidar_time, camera_time)

            if time_diff.nanoseconds < 1e8:  # 100ms tolerance
                # All data is synchronized, perform fusion
                fusion_result = self.perform_sensor_fusion()
                self.publish_fusion_result(fusion_result)

    def perform_sensor_fusion(self):
        # Example fusion: combine IMU orientation with camera object detection
        # and LiDAR obstacle detection for comprehensive scene understanding
        result = {
            'balance_state': self.process_imu_for_balance(),
            'obstacles': self.process_lidar_for_navigation(),
            'objects': self.process_camera_for_interaction()
        }
        return result
```

## Sensor Calibration and Validation

### Calibration Procedures

Proper sensor calibration is essential for humanoid robots:

- **IMU Calibration**: Determine bias, scale factors, and alignment
- **Camera Calibration**: Intrinsic parameters (focal length, distortion) and extrinsic parameters (position relative to robot)
- **LiDAR Calibration**: Alignment with robot coordinate frame

### Validation Techniques

- **Cross-validation**: Compare sensor readings with known references
- **Consistency checks**: Ensure sensor readings are physically plausible
- **Drift detection**: Monitor for sensor degradation over time

## Best Practices for Sensor Integration

1. **Redundancy**: Use multiple sensors for critical functions
2. **Real-time Processing**: Ensure sensor processing meets robot control rates
3. **Noise Handling**: Implement appropriate filtering for sensor noise
4. **Calibration**: Regularly calibrate sensors for accuracy
5. **Synchronization**: Time-align data from multiple sensors
6. **Failure Detection**: Implement sensor health monitoring

## Exercises

1. Implement a simple IMU-based balance controller for a simulated humanoid
2. Create a LiDAR obstacle detection system with visualization
3. Develop a depth camera object detection pipeline for humanoid interaction
4. Design a sensor fusion system that combines all three sensor types
5. Implement sensor calibration routines for a humanoid robot model