---
title: Building ROS 2 Packages (Python)
sidebar_position: 5
description: Creating and structuring ROS 2 packages using Python for humanoid robotics applications
---

# Building ROS 2 Packages (Python)

## Overview

This chapter covers the process of creating ROS 2 packages using Python, which is essential for humanoid robot development and digital twin creation. You'll learn how to structure packages, organize code, handle dependencies, and create proper interfaces for simulation and real-world applications.

## Learning Objectives

By the end of this chapter, you should be able to:

- [ ] Create ROS 2 packages using Python
- [ ] Structure packages following ROS 2 conventions
- [ ] Handle package dependencies properly
- [ ] Organize code for humanoid robot applications
- [ ] Create custom message and service definitions
- [ ] Build and test Python-based ROS 2 packages
- [ ] Integrate packages with simulation environments

## Package Structure Overview

A typical ROS 2 Python package follows this structure:

```
package_name/
├── CMakeLists.txt          # (Not needed for pure Python packages)
├── package.xml             # Package metadata
├── setup.py                # Python package setup
├── setup.cfg               # Installation configuration
├── resource/package_name   # (Only if needed) Resource files
├── test/                   # Test files
├── launch/                 # Launch files
├── config/                 # Configuration files
├── meshes/                 # (If needed) 3D model files
├── worlds/                 # (If needed) Simulation world files
└── package_name/
    ├── __init__.py         # Python package init
    ├── node_name.py        # Python nodes
    └── utils.py            # Utility functions
```

## Creating a New Package

### Using the Command Line Tool

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python humanoid_control_examples
```

This creates a basic package structure with the necessary files.

### Package.xml Content

The `package.xml` file contains package metadata:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>humanoid_control_examples</name>
  <version>0.0.0</version>
  <description>Examples for humanoid robot control</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>message_runtime</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Setup.py Configuration

The `setup.py` file defines how the package is installed:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'humanoid_control_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='Examples for humanoid robot control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_controller = humanoid_control_examples.joint_controller:main',
            'sensor_reader = humanoid_control_examples.sensor_reader:main',
            'motion_planner = humanoid_control_examples.motion_planner:main',
        ],
    },
)
```

### Setup.cfg Configuration

The `setup.cfg` file specifies install directory:

```ini
[develop]
script-dir=$base/lib/humanoid_control_examples

[install]
install-scripts=$base/lib/humanoid_control_examples
```

## Creating Nodes in Python

### Basic Node Structure

```python
# humanoid_control_examples/joint_controller.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        
        # Create publisher for joint commands
        self.joint_cmd_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )
        
        # Create subscriber for joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50 Hz
        
        # Initialize joint state
        self.current_joint_positions = [0.0] * 20  # Assuming 20 DOF humanoid
        
    def joint_state_callback(self, msg):
        self.current_joint_positions = list(msg.position)
        self.get_logger().info(f'Updated joint positions: {self.current_joint_positions[:3]}...')
        
    def control_loop(self):
        # Implement control logic here
        target_pos = self.calculate_target_positions()
        
        cmd_msg = Float64MultiArray()
        cmd_msg.data = target_pos
        self.joint_cmd_publisher.publish(cmd_msg)
        
    def calculate_target_positions(self):
        # Placeholder for control algorithm
        # In a real implementation, this would contain your control logic
        return [0.1, 0.2, 0.3] + [0.0] * 17  # Example target positions

def main(args=None):
    rclpy.init(args=args)
    node = JointController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Custom Message and Service Definitions

### Creating Custom Messages

To create custom messages, create a `msg` directory in your package:

```
humanoid_control_examples/
├── msg/
│   ├── JointCommand.msg
│   └── RobotState.msg
└── srv/
    ├── ExecuteMotion.srv
    └── SetBalanceMode.srv
```

Example message definition (`msg/JointCommand.msg`):

```
# JointCommand.msg
string name
float64[] positions
float64[] velocities
float64[] efforts
```

Example service definition (`srv/ExecuteMotion.srv`):

```
# ExecuteMotion.srv
string motion_name
float64 duration
---
bool success
string message
```

### Using Custom Messages

To use custom messages, you need to add dependencies to your `package.xml`:

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

And update your `setup.py`:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'humanoid_control_examples'

setup(
    # ... other configuration
    data_files=[
        # ... other data files
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
)
```

## Launch Files for Package Execution

Create launch files to run your nodes together:

```python
# launch/humanoid_demo.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joint controller node
        Node(
            package='humanoid_control_examples',
            executable='joint_controller',
            name='joint_controller',
            output='screen',
            parameters=[
                {'control_frequency': 50},
                {'robot_name': 'atlas'}
            ]
        ),
        
        # Sensor reader node
        Node(
            package='humanoid_control_examples',
            executable='sensor_reader',
            name='sensor_reader',
            output='screen'
        ),
        
        # Motion planner node
        Node(
            package='humanoid_control_examples',
            executable='motion_planner',
            name='motion_planner',
            output='screen'
        )
    ])
```

## Dependencies Management

### Ament and Python Dependencies

For Python packages, dependencies are managed through:

1. **setup.py**: Defines Python dependencies and entry points
2. **package.xml**: Defines ROS dependencies (other packages)
3. **requirements.txt**: (Optional) For non-ROS Python packages

### Adding Python Dependencies

Example setup.py with external dependencies:

```python
import os
from glob import glob
from setuptools import setup

package_name = 'humanoid_control_examples'

# Read requirements from requirements.txt
requirements = []
if os.path.exists('requirements.txt'):
    with open('requirements.txt', 'r') as f:
        requirements = f.read().splitlines()

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ... data files configuration
    ],
    install_requires=requirements,  # Include external requirements
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='Examples for humanoid robot control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_controller = humanoid_control_examples.joint_controller:main',
            # ... other entry points
        ],
    },
)
```

### Requirements.txt Example

```
numpy>=1.19.0
scipy>=1.5.0
transforms3d>=0.3.1
control>=0.8.1
```

## Testing Your Package

### Unit Testing with pytest

Create test files in the `test/` directory:

```python
# test/test_joint_controller.py
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from humanoid_control_examples.joint_controller import JointController

class TestJointController(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = JointController()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_creation(self):
        self.assertEqual(self.node.get_name(), 'joint_controller')

if __name__ == '__main__':
    unittest.main()
```

### Running Tests

```bash
cd ~/ros2_ws
colcon build --packages-select humanoid_control_examples
source install/setup.bash
colcon test --packages-select humanoid_control_examples
```

## Package Organization for Humanoid Robots

### Modular Design Principles

For humanoid robots, organize packages by functionality:

```
humanoid_bringup/          # Launch all nodes for the robot
├── launch/
│   └── robot.launch.py
humanoid_control/          # Control algorithms
├── nodes/
│   ├── joint_control.py
│   └── balance_control.py
humanoid_perception/       # Perception algorithms
├── nodes/
│   ├── object_detection.py
│   └── slam.py
humanoid_motion/           # Motion planning
├── nodes/
│   ├── walk_generator.py
│   └── manipulator_planner.py
```

### Configuration Management

Use parameter files for configuration:

```yaml
# config/humanoid_params.yaml
humanoid_control:
  ros__parameters:
    control_frequency: 50
    max_joint_velocity: 2.0
    balance_threshold: 0.05
    joint_names:
      - left_hip
      - left_knee
      - left_ankle
      - right_hip
      # ... all joint names
```

## Simulation Integration

### Packages for Digital Twins

When creating packages for digital twins, consider:

1. **Simulation-specific nodes**: Nodes that behave differently in simulation vs. reality
2. **Parameter configurations**: Different parameters for simulation vs. real robot
3. **Bridge packages**: For connecting to simulation environments (Gazebo, Unity)

### Example Simulation Node

```python
# humanoid_control_examples/sim_joint_controller.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math

class SimJointController(Node):
    def __init__(self):
        super().__init__('sim_joint_controller')
        
        # Publishers and subscribers
        self.joint_cmd_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )
        
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        # Initialize state
        self.current_joint_positions = [0.0] * 20
        self.time = 0.0
        
    def joint_state_callback(self, msg):
        self.current_joint_positions = list(msg.position)
        
    def control_loop(self):
        # Generate a sinusoidal trajectory for simulation
        self.time += 0.02  # Increment time based on timer period
        
        # Example: Move first joint in a sinusoidal pattern
        target_positions = list(self.current_joint_positions)
        target_positions[0] = 0.5 * math.sin(self.time)  # Move first joint
        target_positions[1] = 0.3 * math.sin(self.time * 1.5)  # Move second joint
        
        cmd_msg = Float64MultiArray()
        cmd_msg.data = target_positions
        self.joint_cmd_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimJointController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Building and Installation

### Building the Package

```bash
# Build only this package
colcon build --packages-select humanoid_control_examples

# Build with symlinks to avoid copying files (faster for development)
colcon build --packages-select humanoid_control_examples --symlink-install

# Source the workspace
source install/setup.bash
```

### Running Nodes

```bash
# Run a specific node
ros2 run humanoid_control_examples joint_controller

# Use the executable directly (after sourcing workspace)
joint_controller

# Run with launch file
ros2 launch humanoid_control_examples robot.launch.py
```

## Best Practices

### 1. Use Proper Naming Conventions

- Package names: lowercase with underscores (e.g., humanoid_control_examples)
- Node names: descriptive and unique in the system
- Topic/service names: meaningful and hierarchical

### 2. Handle Parameters Properly

```python
# Good: Declare and use parameters
self.declare_parameter('control_frequency', 50)
self.control_freq = self.get_parameter('control_frequency').value

# Use parameter callbacks for dynamic reconfiguration
self.add_on_set_parameters_callback(self.parameter_callback)

def parameter_callback(self, params):
    for param in params:
        if param.name == 'control_frequency':
            # Adjust timer frequency
            pass
    return SetParametersResult(successful=True)
```

### 3. Implement Proper Error Handling

```python
def control_loop(self):
    try:
        # Control algorithm
        target_pos = self.calculate_target_positions()
        
        if not self.validate_command(target_pos):
            self.get_logger().error('Invalid command, not publishing')
            return
            
        cmd_msg = Float64MultiArray()
        cmd_msg.data = target_pos
        self.joint_cmd_publisher.publish(cmd_msg)
        
    except Exception as e:
        self.get_logger().error(f'Error in control loop: {e}')
```

## Summary

Building ROS 2 packages in Python is fundamental to creating effective humanoid robot applications and their digital twins. Proper package structure, dependency management, and organization are critical for maintainable and scalable systems.

The tools and conventions covered in this chapter will help you create well-structured ROS 2 packages that integrate seamlessly with both simulation environments and real robots.

## Exercises

1. Create a ROS 2 package that publishes a simple topic with joint angles and another that subscribes to it, logging the values.

2. Implement a service in your package that takes joint positions as input and returns whether they're within safe limits.

3. Create a launch file that runs multiple nodes from your package simultaneously.

## Further Reading

- ROS 2 Python Package Development Guide
- "Effective Robotics Programming with ROS" by Anil Mahtani
- ROS 2 Launch System Documentation