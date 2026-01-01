---
title: Launch Files & Parameters
sidebar_position: 6
description: Understanding and creating ROS 2 launch files and parameter configurations for humanoid robotics
---

# Launch Files & Parameters

## Overview

Launch files and parameters are essential tools in ROS 2 for managing complex robot systems. This chapter covers how to create, configure, and use launch files and parameters effectively in humanoid robotics applications and digital twin environments. You'll learn how to streamline the startup and configuration of multiple nodes simultaneously.

## Learning Objectives

By the end of this chapter, you should be able to:

- [ ] Create and configure ROS 2 launch files
- [ ] Use parameters to configure robot behavior
- [ ] Launch multiple nodes with different configurations
- [ ] Configure parameters for simulation vs. real robot scenarios
- [ ] Implement conditional launch elements
- [ ] Pass arguments to launch files
- [ ] Organize launch files for complex humanoid robots

## Introduction to Launch Files

Launch files in ROS 2 allow you to start multiple nodes, configure parameters, and set up the entire system with a single command. This is especially important for humanoid robots, which typically involve numerous interconnected nodes.

### Why Use Launch Files?

For humanoid robots with 20+ joints, multiple sensors, and complex control systems:
- Start all required nodes with a single command
- Ensure proper ordering of node startup
- Configure parameters for each node consistently
- Manage simulation vs. real-robot scenarios
- Enable/disable specific subsystems as needed

## Creating Launch Files

### Basic Launch File Structure

```python
# launch/humanoid_robot.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    
    # Declare launch arguments
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    arg_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='atlas',
        description='Name of the robot'
    )
    
    # Define nodes
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'rate': 50}  # Publish at 50 Hz
        ],
        output='screen'
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': PathJoinSubstitution([
                FindPackageShare('humanoid_description'),
                'urdf',
                [LaunchConfiguration('robot_name'), '.urdf']
            ])}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        arg_use_sim_time,
        arg_robot_name,
        joint_state_publisher,
        robot_state_publisher,
    ])
```

### Running Launch Files

```bash
# Basic launch
ros2 launch humanoid_bringup humanoid_robot.launch.py

# Launch with arguments
ros2 launch humanoid_bringup humanoid_robot.launch.py use_sim_time:=true robot_name:=valkyrie

# Launch specific launch file from package
ros2 launch humanoid_bringup walk_demo.launch.py
```

## Parameter Configuration

Parameters allow you to configure node behavior without changing code.

### Inline Parameters

```python
# Node with inline parameters
controller_node = Node(
    package='humanoid_controller',
    executable='balance_controller',
    name='balance_controller',
    parameters=[
        {'kp': 10.0},
        {'ki': 0.1},
        {'kd': 0.05},
        {'control_frequency': 50},
        {'use_sim_time': use_sim_time}
    ],
    output='screen'
)
```

### YAML Parameter Files

Create parameter files for complex configurations:

```yaml
# config/humanoid_params.yaml
humanoid_controller:
  ros__parameters:
    control_frequency: 50
    max_joint_velocity: 2.0
    balance_threshold: 0.05
    walking_pattern:
      step_height: 0.05
      step_length: 0.3
      step_time: 1.0
    joint_limits:
      hip_max: 1.57
      hip_min: -1.57
      knee_max: 2.0
      knee_min: -0.5
    gravity_compensation: true
    com_height: 0.8
    foot_separation: 0.3

sensor_processor:
  ros__parameters:
    imu_topic: "/imu/data"
    camera_topic: "/camera/image_raw"
    lidar_topic: "/lidar/scan"
    sensor_timeout: 0.1
    enable_filtering: true
```

Using YAML parameters in launch files:

```python
from launch.substitutions import Command
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # ... other configurations
    
    # Get parameter file path
    param_file = PathJoinSubstitution([
        FindPackageShare('humanoid_bringup'),
        'config',
        'humanoid_params.yaml'
    ])
    
    controller_node = Node(
        package='humanoid_controller',
        executable='balance_controller',
        name='balance_controller',
        parameters=[
            param_file,  # Load from YAML file
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        # ... other configurations
        controller_node,
    ])
```

## Launch File Components

### Declare Launch Arguments

```python
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution

# Simple argument
DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation (Gazebo) clock if true'
)

# Argument with choices
DeclareLaunchArgument(
    'robot_type',
    default_value='atlas',
    choices=['atlas', 'valkyrie', 'op3'],
    description='Type of robot to launch'
)

# Argument with default value from substitution
DeclareLaunchArgument(
    'model',
    default_value=PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'urdf',
        'robot.urdf'
    ]),
    description='Robot description file'
)
```

### Conditional Launch Elements

```python
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch argument to control conditional behavior
    launch_gui = LaunchConfiguration('launch_gui')
    
    # Declare the argument
    arg_launch_gui = DeclareLaunchArgument(
        'launch_gui',
        default_value='true',
        description='Launch GUI nodes'
    )
    
    # This node launches only if launch_gui is true
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(launch_gui)
    )
    
    # This node launches only if launch_gui is false
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(launch_gui)
    )
    
    return LaunchDescription([
        arg_launch_gui,
        joint_state_publisher_gui,
        joint_state_publisher,
    ])
```

### Grouping Nodes

```python
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

# Group nodes under a namespace
namespace_group = GroupAction(
    condition=IfCondition(use_namespace),
    actions=[
        PushRosNamespace(namespace=robot_name),
        # All nodes in this group will be under the namespace
        Node(
            package='humanoid_controller',
            executable='balance_controller',
            name='balance_controller'
        ),
        Node(
            package='humanoid_sensor',
            executable='imu_processor',
            name='imu_processor'
        ),
    ]
)
```

## Complex Launch File Example

Here's a complete example for a humanoid robot launch file:

```python
# launch/humanoid_full_system.launch.py
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_gui')
    robot_name = LaunchConfiguration('robot_name')
    namespace = LaunchConfiguration('namespace')
    
    # Declare launch arguments
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    arg_use_gui = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Launch GUI nodes'
    )
    
    arg_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='atlas',
        description='Name of the robot'
    )
    
    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )
    
    # Set global parameters
    set_use_sim_time = SetParameter(
        name='use_sim_time',
        value=use_sim_time
    )
    
    # Controllers launch (assuming separate launch file exists)
    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('humanoid_control'),
                'launch',
                'controllers.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('humanoid_description'),
                'config',
                [robot_name, '_params.yaml']
            ]),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(use_gui),
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    # Joint state publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(use_gui),
        output='screen'
    )
    
    # Sensor processing nodes
    imu_processor = Node(
        package='humanoid_sensor',
        executable='imu_processor',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('humanoid_sensor'),
                'config',
                'imu_config.yaml'
            ])
        ],
        output='screen'
    )
    
    # Main control nodes
    balance_controller = Node(
        package='humanoid_control',
        executable='balance_controller',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('humanoid_control'),
                'config',
                'balance_params.yaml'
            ])
        ],
        output='screen'
    )
    
    walk_generator = Node(
        package='humanoid_motion',
        executable='walk_generator',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('humanoid_motion'),
                'config',
                'walk_params.yaml'
            ])
        ],
        output='screen'
    )
    
    # Group nodes under namespace if specified
    namespaced_nodes = GroupAction(
        condition=IfCondition(namespace),
        actions=[
            PushRosNamespace(namespace=namespace),
            robot_state_publisher,
            joint_state_publisher,
            joint_state_publisher_gui,
            imu_processor,
            balance_controller,
            walk_generator,
        ]
    )
    
    # Return the launch description
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(arg_use_sim_time)
    ld.add_action(arg_use_gui)
    ld.add_action(arg_robot_name)
    ld.add_action(arg_namespace)
    
    # Add global parameter setting
    ld.add_action(set_use_sim_time)
    
    # Add controllers launch
    ld.add_action(controllers_launch)
    
    # Add nodes conditionally based on namespace
    if namespace:
        ld.add_action(namespaced_nodes)
    else:
        ld.add_action(robot_state_publisher)
        ld.add_action(joint_state_publisher)
        ld.add_action(joint_state_publisher_gui)
        ld.add_action(imu_processor)
        ld.add_action(balance_controller)
        ld.add_action(walk_generator)
    
    return ld
```

## Simulation-Specific Launching

For digital twins, you'll often have separate launch files for simulation:

```python
# launch/humanoid_gazebo.launch.py
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from gazebo_ros import GzSimExecutor

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    world = LaunchConfiguration('world')
    
    # Default world
    arg_world = DeclareLaunchArgument(
        'world',
        default_value='empty_world',
        description='Choose one of the world files from `/humanoid_gazebo/worlds`'
    )
    
    # Launch Gazebo simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r -v4 ', world, '.sdf']
        }.items()
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name,
            '-x', '0', '-y', '0', '-z', '1.0'
        ],
        output='screen'
    )
    
    # Robot state publisher for simulation
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'use_sim_time': True},
            PathJoinSubstitution([
                FindPackageShare('humanoid_description'),
                'config',
                [robot_name, '_sim_params.yaml']
            ])
        ],
        output='screen'
    )
    
    # Include robot-specific launch file
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('humanoid_bringup'),
                'launch',
                'humanoid_robot.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'robot_name': robot_name
        }.items()
    )
    
    # Register event handler to start robot after Gazebo starts
    spawn_after_gazebo = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo,
            on_exit=[spawn_robot],
        )
    )
    
    return LaunchDescription([
        arg_world,
        gazebo,
        robot_state_publisher,
        robot_bringup,
        spawn_after_gazebo,
    ])
```

## Best Practices

### 1. Organize Launch Files Logically

Structure your launch files to handle different scenarios:
- `robot.launch.py`: Basic robot bringup
- `simulation.launch.py`: Launch with simulation-specific configurations
- `demo.launch.py`: Launch for demonstrations
- `calibration.launch.py`: Launch for calibration procedures

### 2. Use Launch Arguments Effectively

```python
# Good: Use launch arguments to make your launch files flexible
DeclareLaunchArgument(
    'robot_model',
    default_value='atlas_v5',
    description='Robot model to load'
)

# This allows users to switch between robot models without changing launch files
```

### 3. Separate Configuration Files

Keep parameter configurations separate from launch logic to make them easier to modify:

```
config/
├── robot_params.yaml
├── controller_params.yaml
├── sensor_params.yaml
└── simulation_params.yaml
launch/
├── robot.launch.py
├── simulation.launch.py
└── demo.launch.py
```

### 4. Handle Dependencies Properly

Ensure nodes with dependencies start in the correct order:

```python
# Use event handlers to manage startup order
RegisterEventHandler(
    OnProcessExit(
        target_action=robot_state_publisher,
        on_exit=[controller_manager],
    )
)
```

## Launch File Tips for Humanoid Robotics

### Managing Complex Control Systems

For humanoid robots with multiple controllers:

```python
# Launch controller manager first
controller_manager = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[
        robot_controllers,
        {'use_sim_time': use_sim_time}
    ],
    output='both'
)

# Then launch individual controllers in the right order
controllers = [
    'joint_state_broadcaster',
    'leg_controller',
    'arm_controller',
    'head_controller'
]

for controller in controllers:
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=[controller],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # Add event handler to launch each controller after the previous one
    if controllers.index(controller) > 0:
        prev_controller = controllers[controllers.index(controller) - 1]
        # Register event handler for sequential startup
```

## Summary

Launch files and parameters are essential for managing complex humanoid robot systems. They allow you to start multiple nodes with consistent configurations, handle simulation vs. real-robot scenarios, and create organized, maintainable robot systems.

Properly configured launch files streamline the development and deployment of humanoid robots in both digital twin simulations and real-world applications.

## Exercises

1. Create a launch file that starts a simple robot system with a joint state publisher, robot state publisher, and a custom controller node.

2. Create a YAML parameter file for a humanoid robot with at least 10 configurable parameters.

3. Create a launch file that conditionally launches different nodes based on a launch argument.

## Further Reading

- ROS 2 Launch System Documentation
- "Programming Robots with ROS" by Morgan Quigley
- ROS 2 Parameters Guide