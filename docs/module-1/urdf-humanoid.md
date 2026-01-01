---
title: URDF for Humanoid Robot Modeling
sidebar_position: 4
description: Understanding and creating Universal Robot Description Format (URDF) models for humanoid robots
---

# URDF for Humanoid Robot Modeling

## Overview

Universal Robot Description Format (URDF) is an XML-based format used in ROS to describe robot models. For humanoid robots, URDF is essential for defining the physical structure, kinematic properties, and visual representation of the robot. This chapter covers how to create and work with URDF models specifically designed for humanoid robots.

URDF models enable simulation, visualization, and planning in ROS ecosystems. They define the kinematic and dynamic properties of robots, which are crucial for tasks like motion planning, collision detection, and simulation in environments like Gazebo.

## Learning Objectives

By the end of this chapter, you should be able to:

- [ ] Create basic URDF models for humanoid robots
- [ ] Define kinematic chains with appropriate joint constraints
- [ ] Add visual and collision primitives to robot models
- [ ] Include sensors and actuators in robot descriptions
- [ ] Validate URDF models and troubleshoot common issues

## Understanding URDF Structure

URDF is an XML format that describes a robot model. The main components of a URDF model are:

1. **Links**: Rigid bodies that represent physical parts of the robot
2. **Joints**: Constraints that define how links move relative to each other
3. **Visual**: How the robot appears in visualization tools
4. **Collision**: How the robot behaves in collision detection
5. **Inertial**: Physical properties like mass and moments of inertia

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Links -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>
  
  <!-- Joints -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
  
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>
</robot>
```

## Creating a Simple Humanoid Model

Let's build a basic humanoid model step by step:

### 1. Defining the Robot Root

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- Base/Pelvis Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.15 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.15 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>
```

### 2. Adding the Torso

```xml
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>
  
  <joint name="pelvis_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
  </joint>
```

### 3. Adding a Head

```xml
  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>
  
  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
  </joint>
```

### 4. Adding Arms

```xml
  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.03"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <!-- Joints for left arm -->
  <joint name="torso_to_left_upper_arm" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
  
  <joint name="left_upper_arm_to_lower_arm" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
  
  <!-- Right Arm (mirror of left) -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.03"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <joint name="torso_to_right_upper_arm" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.2 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
  
  <joint name="right_upper_arm_to_lower_arm" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Joint Types in Humanoid Robots

For humanoid robots, different joint types serve various purposes:

### 1. Revolute Joints
- Allow rotation around a single axis
- Used for elbows, knees, shoulders
- Specify limits for safe operation

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="lower_arm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-2.0" upper="2.0" effort="30" velocity="1.0"/>
</joint>
```

### 2. Continuous Joints
- Similar to revolute but with unlimited rotation
- Used for yaw joints in waist/neck

```xml
<joint name="waist_yaw" type="continuous">
  <parent link="pelvis"/>
  <child link="torso"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

### 3. Fixed Joints
- No movement allowed
- Used for attaching sensors or connecting rigid parts

```xml
<joint name="sensor_mount" type="fixed">
  <parent link="head"/>
  <child link="camera_frame"/>
  <origin xyz="0.05 0 0.05" rpy="0 -1.57 0"/>
</joint>
```

## Adding Sensors to URDF

Humanoid robots require various sensors. Here's how to add them:

### 1. Camera (RGBD)
```xml
<link name="camera_frame"/>
<joint name="head_to_camera" type="fixed">
  <parent link="head"/>
  <child link="camera_frame"/>
  <origin xyz="0.05 0 0" rpy="0 0 0"/>
</joint>
```

### 2. IMU
```xml
<link name="imu_link"/>
<joint name="torso_to_imu" type="fixed">
  <parent link="torso"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

### 3. Force/Torque Sensors
```xml
<link name="left_foot_sensor"/>
<joint name="left_ankle_to_left_foot" type="fixed">
  <parent link="left_lower_leg"/>
  <child link="left_foot_sensor"/>
  <origin xyz="0 0 -0.05" rpy="0 0 0"/>
</joint>
```

## Using Xacro for Complex Models

Xacro is a macro language for XML that allows you to create more complex and reusable URDF models:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_humanoid">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.14159"/>
  <xacro:property name="leg_length" value="0.4"/>
  <xacro:property name="arm_length" value="0.3"/>
  
  <!-- Macro for creating links -->
  <xacro:macro name="simple_link" params="name mass length radius *inertia">
    <link name="${name}">
      <visual>
        <geometry>
          <cylinder length="${length}" radius="${radius}"/>
        </geometry>
        <material name="gray">
          <color rgba="0.5 0.5 0.5 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder length="${length}" radius="${radius}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${mass}"/>
        <xacro:insert_block name="inertia"/>
      </inertial>
    </link>
  </xacro:macro>
  
  <!-- Macro for creating joints -->
  <xacro:macro name="simple_joint" params="name type parent child xyz rpy axis">
    <joint name="${name}" type="${type}">
      <parent link="${parent}"/>
      <child link="${child}"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="${axis}"/>
      <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
    </joint>
  </xacro:macro>
  
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.15 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.15 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>
  
  <!-- Torso using macro -->
  <xacro:simple_link name="torso" mass="2.0" length="0.5" radius="0.15">
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
  </xacro:simple_link>
  
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
  </joint>

</robot>
```

## Visualization and Debugging

### 1. Checking the URDF
To validate your URDF, you can use the `check_urdf` command:
```bash
check_urdf /path/to/your/robot.urdf
```

### 2. Visualizing with RViz
Create a launch file to visualize your robot:
```xml
<launch>
  <param name="robot_description" command="$(find xacro)/xacro $(find your_package)/urdf/robot.urdf.xacro" />
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find your_package)/rviz/robot.rviz" />
</launch>
```

### 3. Simulation in Gazebo
To simulate in Gazebo, you need to add Gazebo-specific tags:

```xml
<gazebo reference="base_link">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>

<!-- Add transmission for joints if using controllers -->
<transmission name="left_elbow_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_elbow_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_elbow_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Common Issues and Solutions

### 1. Mass and Inertia Issues
- Always specify mass and inertia for each link
- Use reasonable values based on real components
- For complex shapes, use CAD tools to calculate accurate values

### 2. Kinematic Loop Issues
- URDF doesn't support kinematic loops by default
- For humanoid robots with closed loops (like when both feet touch ground), use specialized controllers or approximations

### 3. Coordinate Frame Issues
- Follow the ROS coordinate frame conventions (X forward, Y left, Z up)
- Use `tf` to broadcast additional frames as needed

## Advanced Topics

### 1. Gazebo Integration
For simulation in Gazebo, additional tags are needed:

```xml
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/humanoid</robotNamespace>
  </plugin>
</gazebo>
```

### 2. Controllers
URDF models work with ROS controllers to enable actual robot movement:

```xml
<ros_control name="PositionJointInterface" type="position">
  <joint name="left_elbow_joint"/>
  <joint name="right_elbow_joint"/>
</ros_control>
```

## Summary

URDF is fundamental for modeling humanoid robots in ROS:

- Links represent rigid bodies of the robot
- Joints define how links move relative to each other
- Visual and collision elements define how the robot appears and interacts
- Sensors can be integrated using appropriate frames
- Xacro helps create complex, reusable models

A well-constructed URDF model is essential for simulation, visualization, motion planning, and control of humanoid robots.

## Exercises

1. Create a URDF model for a simple biped robot with 6 DOF per leg and basic torso.
2. Add a camera and IMU to your humanoid model and verify they are correctly positioned.
3. Convert your URDF to Xacro format to make it more modular and reusable.
4. Research how to properly calculate inertial properties for complex shapes in humanoid robots.

## Further Reading

- URDF/XML Format: http://wiki.ros.org/urdf/XML
- Xacro: http://wiki.ros.org/xacro
- "Programming Robots with ROS" by Quigley et al.
- Gazebo and ROS Integration: http://gazebosim.org/tutorials?tut=ros_overview