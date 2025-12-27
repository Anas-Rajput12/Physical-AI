# URDF Humanoid Body Modeling

## Understanding URDF

Unified Robot Description Format (URDF) is an XML-based format used in ROS to describe robot models. For humanoid robots, URDF defines the physical structure including links (rigid bodies), joints (connections between links), and their properties. URDF is essential for simulation, visualization, and kinematic analysis of humanoid robots.

## URDF Structure for Humanoids

A humanoid robot URDF consists of:

- **Links**: Represent rigid bodies like torso, head, arms, legs
- **Joints**: Define how links connect and move relative to each other
- **Visual**: Defines how the robot appears in simulation
- **Collision**: Defines collision properties for physics simulation
- **Inertial**: Defines mass properties for dynamics simulation

## Basic Humanoid URDF Example

Here's a simplified URDF for a basic humanoid model:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.04"/>
    </inertial>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.0" upper="1.0" effort="100" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Left shoulder joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
  </joint>
</robot>
```

## Advanced Humanoid URDF Components

### Complete Humanoid Skeleton

For a full humanoid model, you would typically include:

- **Trunk**: Pelvis, torso, chest, neck, head
- **Arms**: Shoulder, upper arm, lower arm, hand, fingers
- **Legs**: Hip, thigh, shin, foot, toes

### Joint Types for Humanoid Robots

Different joint types simulate different human movements:

- **Revolute**: Rotational joints (elbows, knees, wrists)
- **Continuous**: Unlimited rotation joints (shoulders, hips)
- **Prismatic**: Linear joints (if needed for telescoping limbs)
- **Fixed**: Non-moving connections (welded parts)

### Example: Hip Joint Complex

```xml
<!-- Hip complex for leg movement -->
<joint name="left_hip_yaw" type="revolute">
  <parent link="pelvis"/>
  <child link="left_thigh"/>
  <origin xyz="0 -0.1 -0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-0.5" upper="0.5" effort="200" velocity="1.5"/>
</joint>

<joint name="left_hip_roll" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_thigh_lower"/>
  <origin xyz="0 0 -0.2" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-0.4" upper="1.0" effort="200" velocity="1.5"/>
</joint>

<joint name="left_hip_pitch" type="revolute">
  <parent link="left_thigh_lower"/>
  <child link="left_shin"/>
  <origin xyz="0 0 -0.2" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="0.5" effort="200" velocity="1.5"/>
</joint>
```

## Gazebo-Specific Extensions

For simulation in Gazebo, you can add Gazebo-specific tags:

```xml
<gazebo reference="torso">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
</gazebo>

<!-- Transmission for actuator control -->
<transmission name="left_shoulder_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_shoulder_joint">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_shoulder_motor">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Xacro for Complex Humanoids

For complex humanoid models, use Xacro (XML Macros) to avoid repetition:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="arm_length" value="0.3" />
  <xacro:property name="arm_radius" value="0.05" />

  <!-- Macro for arm -->
  <xacro:macro name="arm" params="side reflect">
    <link name="${side}_upper_arm">
      <visual>
        <geometry>
          <cylinder length="${arm_length}" radius="${arm_radius}"/>
        </geometry>
        <origin xyz="0 0 ${arm_length/2}" rpy="0 0 0"/>
      </visual>
      <collision>
        <geometry>
          <cylinder length="${arm_length}" radius="${arm_radius}"/>
        </geometry>
        <origin xyz="0 0 ${arm_length/2}" rpy="0 0 0"/>
      </collision>
      <inertial>
        <mass value="1.5"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:arm side="left" reflect="1"/>
  <xacro:arm side="right" reflect="-1"/>

</robot>
```

## Best Practices for Humanoid URDF

1. **Mass Distribution**: Accurately model mass properties for realistic simulation
2. **Joint Limits**: Set realistic joint limits based on human anatomy
3. **Inertia Tensors**: Calculate proper inertia tensors for stable simulation
4. **Visual vs Collision**: Use simplified geometries for collision to improve performance
5. **Origin Conventions**: Follow consistent coordinate frame conventions
6. **Mesh Models**: Use high-quality mesh models for visual representation
7. **Transmission Definitions**: Include proper transmission definitions for control

## Common Humanoid URDF Challenges

- **Kinematic Loops**: Humanoid models often have closed kinematic chains (e.g., when hands touch)
- **Balance Simulation**: Accurate modeling required for stable bipedal simulation
- **Computational Complexity**: Full humanoid models can be computationally expensive
- **Realism vs Performance**: Balance between realistic modeling and simulation performance

## Exercises

1. Create a complete URDF model for a simple humanoid with at least 12 degrees of freedom
2. Add Gazebo plugins for physics simulation and visualization
3. Use Xacro to create parametric humanoid models of different sizes
4. Implement collision avoidance constraints in your URDF model