---
title: URDF & SDF Formats
sidebar_position: 9
description: Understanding URDF and SDF formats for robot modeling in ROS 2 and Gazebo simulation
---

# URDF & SDF Formats

## Overview

Universal Robot Description Format (URDF) and Simulation Description Format (SDF) are XML-based formats used to describe robots in ROS and Gazebo respectively. Understanding these formats is crucial for humanoid robotics as they define the physical structure, kinematics, dynamics, and sensors of your robot. This chapter covers both formats, their relationship, and how to create effective models for digital twins in humanoid robotics.

## Learning Objectives

By the end of this chapter, you should be able to:

- [ ] Understand the structure and components of URDF and SDF files
- [ ] Create URDF files for complex humanoid robots
- [ ] Convert between URDF and SDF for simulation
- [ ] Include Gazebo-specific elements in URDF files
- [ ] Create SDF files for direct use in Gazebo
- [ ] Validate and debug robot description files
- [ ] Optimize models for simulation performance

## Introduction to URDF (Universal Robot Description Format)

### What is URDF?

URDF (Unified Robot Description Format) is an XML format used in ROS to describe robot models. It defines the physical and kinematic properties of robots, including:

- Links (rigid bodies)
- Joints (connections between links)
- Visual and collision properties
- Inertial properties
- Sensors and actuators
- Transmission information

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Links definition -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.6"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  
  <!-- Joint definition -->
  <joint name="hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_leg"/>
    <origin xyz="0 0.1 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>
  
  <!-- Child link -->
  <link name="left_leg">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="orange">
        <color rgba="1 0.5 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
</robot>
```

## Links in URDF

### Link Components

A link in URDF represents a rigid body and contains three main elements:

1. **Visual**: Defines how the link looks in simulation and visualization tools
2. **Collision**: Defines the collision geometry for physics simulation
3. **Inertial**: Defines mass properties for dynamics simulation

### Visual Element

```xml
<link name="link_name">
  <visual>
    <!-- Origin relative to joint -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    
    <!-- Geometry definition -->
    <geometry>
      <!-- Box -->
      <box size="1.0 0.5 0.3"/>
      
      <!-- Cylinder -->
      <cylinder radius="0.1" length="0.5"/>
      
      <!-- Sphere -->
      <sphere radius="0.2"/>
      
      <!-- Mesh -->
      <mesh filename="package://humanoid_description/meshes/leg.dae" scale="1.0 1.0 1.0"/>
    </geometry>
    
    <!-- Material -->
    <material name="red">
      <color rgba="1 0 0 1"/>
      <!-- Or reference texture -->
      <!-- <texture filename="path/to/texture.png"/> -->
    </material>
  </visual>
</link>
```

### Collision Element

```xml
<link name="link_name">
  <collision>
    <!-- Origin relative to joint -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    
    <!-- Collision geometry -->
    <geometry>
      <!-- Often simpler than visual geometry for performance -->
      <box size="1.0 0.5 0.3"/>
    </geometry>
  </collision>
</link>
```

### Inertial Element

```xml
<link name="link_name">
  <inertial>
    <!-- Mass of the link -->
    <mass value="1.0"/>
    
    <!-- Inertia matrix -->
    <!-- For a box: 
         ixx = 1/12 * m * (h*h + d*d)
         iyy = 1/12 * m * (w*w + d*d) 
         izz = 1/12 * m * (w*w + h*h) -->
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
  </inertial>
</link>
```

## Joints in URDF

### Joint Types

URDF supports several types of joints:

1. **Fixed**: No degrees of freedom (used to connect rigidly)
2. **Revolute**: Single axis of rotation with limits
3. **Continuous**: Single axis of rotation without limits
4. **Prismatic**: Single axis of translation with limits
5. **Planar**: Motion on a plane
6. **Floating**: Six degrees of freedom

### Joint Definition

```xml
<joint name="joint_name" type="revolute">
  <!-- Parent link -->
  <parent link="parent_link_name"/>
  
  <!-- Child link -->
  <child link="child_link_name"/>
  
  <!-- Transformation from parent to child -->
  <origin xyz="1.0 0 0" rpy="0 0 0"/>
  
  <!-- Axis of rotation/translation -->
  <axis xyz="0 0 1"/>
  
  <!-- Joint limits (for revolute and prismatic joints) -->
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  
  <!-- Dynamics properties -->
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

### Joint Examples for Humanoid Robots

```xml
<!-- Hip joint (revolute) -->
<joint name="left_hip_yaw" type="revolute">
  <parent link="torso"/>
  <child link="left_pelvis"/>
  <origin xyz="0 -0.15 -0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.0" upper="1.0" effort="200" velocity="2.0"/>
  <dynamics damping="1.0" friction="0.2"/>
</joint>

<!-- Knee joint (revolute) -->
<joint name="left_knee" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_shin"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="2.5" effort="250" velocity="1.5"/>
  <dynamics damping="1.5" friction="0.3"/>
</joint>

<!-- Fixed joint for sensor mounting -->
<joint name="imu_mount" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
</joint>
```

## URDF for Humanoid Robots

### Humanoid Robot Skeleton

A humanoid robot typically has a skeleton with multiple chains:

```
base_link (torso)
├── head
│   └── camera_link
├── left_arm
│   ├── left_shoulder
│   ├── left_elbow
│   └── left_wrist
│       └── left_gripper
├── right_arm
│   ├── right_shoulder
│   ├── right_elbow
│   └── right_wrist
│       └── right_gripper
├── left_leg
│   ├── left_hip
│   ├── left_knee
│   └── left_ankle
│       └── left_foot
└── right_leg
    ├── right_hip
    ├── right_knee
    └── right_ankle
        └── right_foot
```

### Complete Humanoid URDF Example

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Include common definitions -->
  <xacro:include filename="$(find humanoid_description)/urdf/materials.urdf.xacro"/>
  
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>
  
  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1.0"/>
  </joint>
  
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>
  
  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0.15 0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1.0"/>
  </joint>
  
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <!-- More joints and links for complete humanoid would follow... -->
</robot>
```

## Introduction to SDF (Simulation Description Format)

### What is SDF?

SDF (Simulation Description Format) is an XML format used by Gazebo (and its underlying Ignition libraries) to describe simulation environments, robots, and objects. Unlike URDF, SDF is specifically designed for simulation and includes more detailed physics and sensor information.

### Basic SDF Structure

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="humanoid_robot">
    <!-- Links -->
    <link name="base_link">
      <!-- Inertial properties -->
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1.0</iyy>
          <iyz>0.0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>
      
      <!-- Visual properties -->
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.3 0.6</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 0.8 1</ambient>
          <diffuse>0 0 0.8 1</diffuse>
        </material>
      </visual>
      
      <!-- Collision properties -->
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.3 0.6</size>
          </box>
        </geometry>
      </collision>
    </link>
    
    <!-- Joints -->
    <joint name="hip_joint" type="revolute">
      <parent>base_link</parent>
      <child>left_leg</child>
      <pose>0 0.1 -0.3 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>100</effort>
          <velocity>1.0</velocity>
        </limit>
      </axis>
    </joint>
    
    <!-- More links and joints can be added -->
  </model>
</sdf>
```

## Converting URDF to SDF

### Using Robot Description Parameter

Gazebo can read URDF directly and convert it to SDF internally:

```bash
# If you have a robot_description parameter set
ign gazebo -r -v4 --ros-args -p robot_description:='$(find my_robot_description)/urdf/robot.urdf'
```

### Using Command Line Tools

Convert URDF to SDF using the `gz sdf` command:

```bash
# Convert URDF to SDF
gz sdf -p /path/to/robot.urdf > /path/to/robot.sdf

# Convert and validate
gz sdf -k /path/to/robot.urdf
```

## Adding Gazebo-Specific Elements to URDF

When using URDF with Gazebo, you can add Gazebo-specific elements:

```xml
<!-- Gazebo materials -->
<gazebo reference="base_link">
  <material>Gazebo/Blue</material>
  <!-- Additional physics properties -->
  <mu1>0.9</mu1>
  <mu2>0.9</mu2>
  <self_collide>false</self_collide>
  <gravity>true</gravity>
</gazebo>

<!-- Gazebo plugins -->
<gazebo>
  <plugin name="ros2_control" filename="libgazebo_ros2_control.so">
    <parameters>$(find my_robot_description)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>

<!-- Gazebo sensors -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <camera name="head_camera">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>300</far>
      </clip>
    </camera>
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

## URDF/Xacro for Complex Models

### Introduction to Xacro

Xacro (XML Macros) extends URDF with macros, properties, and mathematical expressions, making it easier to create and maintain complex models.

### Basic Xacro Elements

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">
  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="robot_height" value="1.7" />
  <xacro:property name="link_length" value="0.4" />
  
  <!-- Macros -->
  <xacro:macro name="simple_cylinder" params="name radius length mass xyz:='0 0 0' rpy:='0 0 0'">
    <link name="${name}">
      <visual>
        <origin xyz="${xyz}" rpy="${rpy}" />
        <geometry>
          <cylinder radius="${radius}" length="${length}" />
        </geometry>
        <material name="light_grey">
          <color rgba="0.8 0.8 0.8 1.0" />
        </material>
      </visual>
      <collision>
        <origin xyz="${xyz}" rpy="${rpy}" />
        <geometry>
          <cylinder radius="${radius}" length="${length}" />
        </geometry>
      </collision>
      <inertial>
        <mass value="${mass}" />
        <inertia
          ixx="${0.0833333 * mass * (3*radius*radius + length*length)}"
          ixy="0" ixz="0"
          iyy="${0.0833333 * mass * (3*radius*radius + length*length)}"
          iyz="0"
          izz="${0.5 * mass * radius * radius}" />
      </inertial>
    </link>
  </xacro:macro>
  
  <!-- Using the macro -->
  <xacro:simple_cylinder name="upper_arm" radius="0.05" length="0.3" mass="1.5" />
</robot>
```

### Humanoid Leg Macro Example

```xml
<!-- Define a macro for humanoid leg -->
<xacro:macro name="humanoid_leg" params="side parent xyz rpy">
  <!-- Hip joint -->
  <joint name="${side}_hip_yaw_joint" type="revolute">
    <parent link="${parent}"/>
    <child link="${side}_hip_yaw_link"/>
    <origin xyz="${xyz}" rpy="${rpy}"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="200" velocity="2.0"/>
    <dynamics damping="1.0" friction="0.2"/>
  </joint>
  
  <link name="${side}_hip_yaw_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.0025"/>
    </inertial>
  </link>
  
  <!-- Additional joints and links for complete leg would follow -->
</xacro:macro>

<!-- Use the macro to create both legs -->
<xacro:humanoid_leg side="left" parent="torso" xyz="-0.05 -0.15 -0.3" rpy="0 0 0"/>
<xacro:humanoid_leg side="right" parent="torso" xyz="-0.05 0.15 -0.3" rpy="0 0 0"/>
```

## Sensors in URDF/SDF

### IMU Sensor

```xml
<!-- Fixed joint to mount IMU -->
<joint name="imu_joint" type="fixed">
  <parent link="torso"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>

<link name="imu_link">
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
  </inertial>
</link>

<!-- Gazebo plugin for IMU -->
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>false</visualize>
    <topic>__default_topic__</topic>
    <ignition_frame_id>imu_link</ignition_frame_id>
    
    <!-- IMU noise parameters -->
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

### Camera Sensor

```xml
<!-- Fixed joint to mount camera -->
<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0" rpy="0 0 0"/>
</joint>

<link name="camera_link">
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
  </inertial>
</link>

<!-- Gazebo camera plugin -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <camera name="head_camera">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>300</far>
      </clip>
    </camera>
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

## Validation and Debugging

### Validating URDF Files

```bash
# Check if URDF is syntactically correct
check_urdf /path/to/robot.urdf

# Print URDF information
urdf_to_graphiz /path/to/robot.urdf

# Use xacro to check for errors
xacro -o output.urdf /path/to/robot.urdf.xacro
```

### Visualizing URDF

```bash
# Use robot state publisher to visualize
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat robot.urdf)'

# Use rviz to visualize
ros2 run rviz2 rviz2
# Then add RobotModel display and set topic to robot_description
```

### Common URDF Issues

1. **Invalid joint connections**: Make sure all joints have valid parent and child links
2. **Missing inertial properties**: Every link needs mass and inertia for dynamic simulation
3. **Incorrect origins**: Joint origins must properly connect parent and child links
4. **Over-constrained systems**: Avoid creating kinematic loops without proper consideration

## Performance Optimization

### Simplifying for Simulation

For better simulation performance:

1. **Simplify collision geometry**: Use boxes instead of complex meshes for collision detection
2. **Reduce link count**: Combine multiple visual elements into single links when possible
3. **Optimize mesh resolution**: Use lower-resolution meshes for collision but higher-resolution for visualization
4. **Appropriate update rates**: Match physics update rate to control system requirements

### Hierarchical Modeling

Structure complex humanoid models hierarchically:

```xml
<!-- Use xacro includes for modularity -->
<xacro:include filename="$(find humanoid_description)/urdf/arms.urdf.xacro"/>
<xacro:include filename="$(find humanoid_description)/urdf/legs.urdf.xacro"/>
<xacro:include filename="$(find humanoid_description)/urdf/head.urdf.xacro"/>
<xacro:include filename="$(find humanoid_description)/urdf/torso.urdf.xacro"/>
```

## Digital Twin Considerations

### Model Fidelity vs. Performance

For digital twins, balance model fidelity with computational performance:

- **High fidelity for visualization**: Use detailed meshes for presentation
- **Simplified for physics**: Use simple shapes for collision and dynamics
- **Accurate mass properties**: Ensure inertial parameters match the real robot
- **Realistic sensor models**: Include appropriate noise models for sensors

### Synchronization with Real Robot

Keep simulation models synchronized with real robot changes:

- Maintain version control for robot descriptions
- Update simulation parameters based on real robot calibration
- Regular validation of simulation against real robot behavior

## Summary

URDF and SDF are fundamental formats for describing robots in ROS and Gazebo. Understanding these formats is crucial for creating effective digital twins of humanoid robots. Properly structured URDF/SDF files enable accurate simulation, visualization, and control of complex humanoid systems.

Using Xacro macros and includes helps manage complex robot models efficiently. Adding appropriate sensors and Gazebo plugins enables comprehensive simulation of humanoid robots for digital twin applications.

## Exercises

1. Create a simple URDF file for a 6-DOF arm and validate it using check_urdf.

2. Modify the arm URDF to include a camera sensor using Gazebo plugins.

3. Create an Xacro macro for a humanoid leg and instantiate both left and right legs.

4. Convert a simple URDF to SDF and compare the results.

## Further Reading

- URDF Documentation
- SDF Documentation
- Xacro Tutorials
- ROS Robot Description Best Practices
- Gazebo Model Tutorial