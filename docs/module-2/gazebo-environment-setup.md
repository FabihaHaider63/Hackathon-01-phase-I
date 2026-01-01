---
title: Gazebo Environment Setup
sidebar_position: 8
description: Setting up Gazebo simulation environments for humanoid robotics applications
---

# Gazebo Environment Setup

## Overview

Setting up Gazebo simulation environments is a critical component of developing digital twins for humanoid robotics. This chapter covers the process of configuring Gazebo for humanoid robot simulation, including installation, basic environment setup, integration with ROS 2, and advanced configuration options for realistic humanoid simulation.

## Learning Objectives

By the end of this chapter, you should be able to:

- [ ] Install and configure Gazebo for humanoid robotics applications
- [ ] Set up basic simulation environments in Gazebo
- [ ] Integrate Gazebo with ROS 2 using the appropriate packages
- [ ] Configure basic robot models for Gazebo simulation
- [ ] Launch humanoid robots in simulation environments
- [ ] Verify and troubleshoot Gazebo integration

## Installing Gazebo

### Prerequisites

Before installing Gazebo, ensure you have ROS 2 installed. For this course, we'll use ROS 2 Humble Hawksbill (which has long-term support).

### Installation Options

Gazebo has evolved, and you have several options depending on your ROS 2 version:

#### Option 1: Install Ignition Gazebo (Recommended for ROS 2 Humble)

```bash
# Update package lists
sudo apt update

# Install Ignition Fortress (Gazebo) with ROS 2 bridge
sudo apt install ignition-fortress ros-humble-ros-gz ros-humble-ros-gz-bridge ros-humble-ros-gz-common ros-humble-ros-gz-sim ros-humble-ros-gz-sim-msgs

# Verify installation
ign gazebo --version
```

#### Option 2: Install Classic Gazebo (For older setups)

```bash
# Install classic Gazebo
sudo apt install gazebo libgazebo-dev

# Install ROS 2 Gazebo packages
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-plugins ros-humble-gazebo-ros-pkgs
```

### Verification

Test the installation by launching Gazebo:

```bash
# Launch Gazebo GUI
ign gazebo

# Or launch with an empty world
ign gazebo empty.sdf
```

## Basic Gazebo Environment Configuration

### World Files

Gazebo environments are defined using SDF (Simulation Description Format) world files. For humanoid robotics, we typically need more complex environments than the default empty world.

#### Creating a Basic World File

Create a world file for humanoid simulation at `humanoid_gazebo/worlds/humanoid_empty.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_empty">
    <!-- Include ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add a simple box for testing -->
    <model name="test_box">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="box_link">
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 1.0 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.3 0.3 1</ambient>
            <diffuse>0.8 0.3 0.3 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1.0 1.0 1.0</size>
            </box>
          </geometry>
        </collision>
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
      </link>
    </model>

    <!-- Physics configuration -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

### Launching Custom Worlds

```bash
# Launch with custom world
ign gazebo -r /path/to/your/world/file.sdf

# Or copy to Gazebo models directory and reference by name
ign gazebo humanoid_empty.sdf
```

## ROS 2 Integration Setup

### Required ROS 2 Packages

For proper Gazebo-ROS 2 integration, install the necessary packages:

```bash
sudo apt install ros-humble-ros-gz ros-humble-ros-gz-bridge ros-humble-ros-gz-common ros-humble-ros-gz-sim ros-humble-ros-gz-sim-msgs
```

### Gazebo ROS2 Control Package

For controlling humanoid robots in simulation, you'll need the ros2_control packages:

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control
```

## Creating a Gazebo Package for Humanoid Simulation

### Package Structure

Create a package specifically for Gazebo simulation of your humanoid robot:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake humanoid_gazebo --dependencies rclcpp std_msgs sensor_msgs geometry_msgs gazebo_ros2_control
```

Update the package.xml:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>humanoid_gazebo</name>
  <version>0.0.0</version>
  <description>Gazebo simulation for humanoid robots</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>gazebo_ros2_control</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Directory Structure

```
humanoid_gazebo/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── humanoid_gazebo.launch.py
├── worlds/
│   ├── humanoid_empty.sdf
│   └── humanoid_obstacles.sdf
├── models/
│   └── humanoid_robot
│       ├── model.sdf
│       └── meshes/
└── config/
    ├── robot_controllers.yaml
    └── gazebo_params.yaml
```

## Launching Humanoid Robots in Gazebo

### ROS 2 Launch File for Gazebo

Create a launch file to start Gazebo with your humanoid robot:

```python
# launch/humanoid_gazebo.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    pose_x = LaunchConfiguration('pose_x')
    pose_y = LaunchConfiguration('pose_y')
    pose_z = LaunchConfiguration('pose_z')
    
    # Declare launch arguments
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    arg_world = DeclareLaunchArgument(
        'world',
        default_value='humanoid_empty.sdf',
        description='Choose one of the world files from `/humanoid_gazebo/worlds`'
    )
    
    arg_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid_robot',
        description='Name of the robot to spawn'
    )
    
    arg_pose_x = DeclareLaunchArgument(
        'pose_x',
        default_value='0.0',
        description='Initial x position of the robot'
    )
    
    arg_pose_y = DeclareLaunchArgument(
        'pose_y',
        default_value='0.0',
        description='Initial y position of the robot'
    )
    
    arg_pose_z = DeclareLaunchArgument(
        'pose_z',
        default_value='1.0',
        description='Initial z position of the robot'
    )
    
    # Start Gazebo with the specified world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r -v4 ', world]
        }.items()
    )
    
    # Robot State Publisher
    robot_description = PathJoinSubstitution([
        FindPackageShare('humanoid_description'),
        'urdf',
        'humanoid.urdf.xacro'
    ])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': PathJoinSubstitution([
                FindPackageShare('humanoid_description'),
                'urdf',
                'humanoid.urdf.xacro'
            ])}
        ]
    )
    
    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-topic', 'robot_description',
            '-x', pose_x,
            '-y', pose_y,
            '-z', pose_z
        ],
        output='screen'
    )
    
    # Bridge for ROS 2 to Gazebo communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFArray@ignition.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU'
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        arg_use_sim_time,
        arg_world,
        arg_robot_name,
        arg_pose_x,
        arg_pose_y,
        arg_pose_z,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
    ])
```

## Robot Configuration for Gazebo

### URDF with Gazebo Elements

Your robot's URDF needs to include Gazebo-specific elements for proper simulation. Here's an example:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">
  <!-- Include controller configurations -->
  <xacro:include filename="$(find humanoid_description)/urdf/materials.urdf.xacro"/>
  <xacro:include filename="$(find humanoid_description)/urdf/humanoid.transmission.xacro"/>
  
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.3"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.3 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Hip joint and link -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_hip"/>
    <origin xyz="0 0.15 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>
  
  <link name="left_hip">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.15"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Gazebo-specific elements -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
    <self_collide>false</self_collide>
    <gravity>true</gravity>
  </gazebo>
  
  <gazebo reference="left_hip">
    <material>Gazebo/Red</material>
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
    <self_collide>false</self_collide>
    <gravity>true</gravity>
  </gazebo>
  
  <!-- Gazebo ROS2 Control plugin -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find humanoid_description)/config/humanoid_controllers.yaml</parameters>
    </plugin>
  </gazebo>
</robot>
```

### Controller Configuration

Create controller configuration file: `config/robot_controllers.yaml`:

```yaml
# Controller configuration for humanoid robot
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    left_leg_controller:
      type: position_controllers/JointGroupPositionController

    right_leg_controller:
      type: position_controllers/JointGroupPositionController

    left_arm_controller:
      type: position_controllers/JointGroupPositionController

    right_arm_controller:
      type: position_controllers/JointGroupPositionController

left_leg_controller:
  ros__parameters:
    joints:
      - left_hip_joint
      - left_knee_joint
      - left_ankle_joint

right_leg_controller:
  ros__parameters:
    joints:
      - right_hip_joint
      - right_knee_joint
      - right_ankle_joint

left_arm_controller:
  ros__parameters:
    joints:
      - left_shoulder_joint
      - left_elbow_joint

right_arm_controller:
  ros__parameters:
    joints:
      - right_shoulder_joint
      - right_elbow_joint
```

## Running the Simulation

### Build and Source the Workspace

```bash
cd ~/ros2_ws
colcon build --packages-select humanoid_gazebo humanoid_description
source install/setup.bash
```

### Launch the Simulation

```bash
# Launch with default settings
ros2 launch humanoid_gazebo humanoid_gazebo.launch.py

# Launch with custom world
ros2 launch humanoid_gazebo humanoid_gazebo.launch.py world:=humanoid_obstacles.sdf

# Launch with pose offset
ros2 launch humanoid_gazebo humanoid_gazebo.launch.py pose_x:=2.0 pose_y:=1.0
```

## Environment Customization

### Adding Objects and Obstacles

Create more complex environments by adding objects and obstacles to your world files:

```xml
<!-- Add a platform for the robot to walk on -->
<model name="platform">
  <pose>0 0 0 0 0 0</pose>
  <link name="platform_link">
    <visual name="visual">
      <geometry>
        <box>
          <size>4.0 4.0 0.1</size>
        </box>
      </geometry>
      <material>
        <ambient>0.5 0.5 0.5 1</ambient>
        <diffuse>0.5 0.5 0.5 1</diffuse>
      </material>
    </visual>
    <collision name="collision">
      <geometry>
        <box>
          <size>4.0 4.0 0.1</size>
        </box>
      </geometry>
    </collision>
    <inertial>
      <mass>100.0</mass>
      <inertia>
        <ixx>100.0</ixx>
        <ixy>0.0</ixy>
        <ixz>0.0</ixz>
        <iyy>100.0</iyy>
        <iyz>0.0</iyz>
        <izz>100.0</izz>
      </inertia>
    </inertial>
  </link>
</model>

<!-- Add some obstacles -->
<model name="obstacle_1">
  <pose>3 2 0.5 0 0 0</pose>
  <link name="obstacle_1_link">
    <visual name="visual">
      <geometry>
        <box>
          <size>0.5 0.5 1.0</size>
        </box>
      </geometry>
      <material>
        <ambient>0.8 0.6 0.1 1</ambient>
        <diffuse>0.8 0.6 0.1 1</diffuse>
      </material>
    </visual>
    <collision name="collision">
      <geometry>
        <box>
          <size>0.5 0.5 1.0</size>
        </box>
      </geometry>
    </collision>
    <inertial>
      <mass>5.0</mass>
      <inertia>
        <ixx>0.5</ixx>
        <ixy>0.0</ixy>
        <ixz>0.0</ixz>
        <iyy>0.5</iyy>
        <iyz>0.0</iyz>
        <izz>0.5</izz>
      </inertia>
    </inertial>
  </link>
</model>
```

### Custom Models

You can use custom models by placing them in the Gazebo models directory or your package:

```bash
# Create models directory
mkdir -p ~/ros2_ws/src/humanoid_gazebo/models/my_custom_room

# Add model.config file
echo '<?xml version="1.0"?>
<model>
  <name>my_custom_room</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>
  <description>A custom room for humanoid robot testing</description>
</model>' > ~/ros2_ws/src/humanoid_gazebo/models/my_custom_room/model.config

# Add model.sdf file with your room definition
```

## Advanced Environment Setup

### Physics Configuration

Fine-tune physics parameters for realistic humanoid simulation:

```xml
<!-- In your world file -->
<physics name="humanoid_physics" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
  <ode>
    <solver>
      <type>quick</type>
      <iters>100</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### Lighting and Visualization

Configure lighting for better visualization of humanoid robots:

```xml
<!-- Add to your world file -->
<light name="sun" type="directional">
  <cast_shadows>true</cast_shadows>
  <pose>0 0 10 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.9 0.9 0.9 1</specular>
  <attenuation>
    <range>1000</range>
    <constant>0.9</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
  <direction>-0.3 0.2 -0.9</direction>
</light>
```

## Troubleshooting Common Issues

### Model Not Loading

If your robot model isn't loading in Gazebo:

1. Verify the URDF is valid:
```bash
check_urdf /path/to/your/robot.urdf
```

2. Check that all mesh files are accessible:
```bash
# Make sure Gazebo can find your meshes
echo $GAZEBO_MODEL_PATH
```

### Physics Problems

If the humanoid robot behaves unrealistically:

1. Check mass and inertia values in your URDF
2. Verify joint limits and friction parameters
3. Adjust physics parameters in the world file
4. Increase the update rate if needed (but be aware of performance impact)

### ROS 2 Bridge Issues

If ROS 2 topics aren't connecting to Gazebo:

1. Verify bridge parameters are correct
2. Check that both ROS 2 and Gazebo are using the same time source
3. Ensure the bridge node is running

## Performance Optimization

### Optimizing for Real-time Simulation

For real-time humanoid robot simulation:

1. **Reduce visual complexity**: Use simple shapes for collision but complex shapes for visualization when needed
2. **Optimize update rates**: Match physics update rate to controller needs (typically 1000Hz for humanoid control)
3. **Simplify collision geometry**: Use simpler shapes for collision detection than for visualization
4. **Limit number of contacts**: Reduce the number of contact points between complex surfaces

### Multi-threading Configuration

In your launch file, consider configuration for better performance:

```python
# Add execution context configuration for performance
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# Use composable nodes for better performance in some cases
container = ComposableNodeContainer(
    name='simulation_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(
            package='robot_state_publisher',
            plugin='robot_state_publisher::RobotStatePublisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ],
    output='both',
)
```

## Summary

Setting up Gazebo for humanoid robot simulation requires careful configuration of the simulation environment, robot models, and ROS 2 integration. Following the steps in this chapter will enable you to create a robust simulation environment that serves as an effective digital twin for your humanoid robot development.

Proper simulation setup is crucial for developing and testing humanoid robot control algorithms before deployment on physical hardware, making Gazebo an essential tool in the humanoid robotics workflow.

## Exercises

1. Install Gazebo and verify the installation by launching a simple world.

2. Create a basic world file with some obstacles for humanoid navigation testing.

3. Set up a launch file that starts Gazebo with a simple robot model.

4. Configure basic controllers for a humanoid robot model in Gazebo.

## Further Reading

- Gazebo Installation Guide
- ROS 2 Gazebo Integration Documentation
- Ignition Gazebo Tutorials
- Creating Custom Worlds in Gazebo