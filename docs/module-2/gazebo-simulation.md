---
title: Gazebo Simulation (Weeks 6–7)
sidebar_position: 7
description: Setting up and using Gazebo for humanoid robot simulation in the digital twin workflow
---

# Gazebo Simulation (Weeks 6–7)

## Overview

Gazebo is a powerful robot simulation environment that provides accurate physics simulation, realistic rendering, and integration with ROS 2. In the context of digital twins for humanoid robotics, Gazebo serves as a critical bridge between design and deployment, allowing for extensive testing of control algorithms in a safe, controlled virtual environment before implementation on physical hardware.

## Learning Objectives

By the end of this chapter, you should be able to:

- [ ] Install and configure Gazebo for humanoid robot simulation
- [ ] Create and modify robot models (URDF) for Gazebo simulation
- [ ] Set up physics parameters for realistic humanoid simulation
- [ ] Configure sensors (LiDAR, cameras, IMUs) in Gazebo
- [ ] Integrate Gazebo with ROS 2 using the Gazebo ROS2 package
- [ ] Simulate complex humanoid behaviors in Gazebo
- [ ] Validate simulation results against real-world expectations

## Weekly Breakdown

This chapter covers material intended for Weeks 6-7 of the course:

- **Week 6**: Gazebo environment setup, URDF/SDF integration, basic physics simulation
- **Week 7**: Sensor simulation, advanced physics, and human-robot interaction in simulation

## Introduction to Gazebo

### What is Gazebo?

Gazebo is a 3D dynamic simulator with the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. For humanoid robotics, Gazebo provides:

- **Accurate Physics Simulation**: Realistic simulation of dynamics, kinematics, and contact forces
- **High-Quality Rendering**: Visualization of robot behavior in detailed environments
- **Extensive Sensor Simulation**: Support for cameras, LiDAR, IMUs, force/torque sensors, etc.
- **Robust Plugin System**: Extensibility through custom plugins
- **ROS Integration**: Seamless integration with ROS/ROS 2 for robot control

### Gazebo vs. Other Simulation Environments

Compared to alternatives like PyBullet or MuJoCo, Gazebo offers:
- Better visualization and rendering capabilities
- More comprehensive sensor simulation
- Strong ROS integration
- Open-source with active development community
- Support for complex environments and multi-robot scenarios

## Installing and Setting Up Gazebo

### Installation

Gazebo has been restructured into the Ignition suite of tools. For ROS 2 Humble and newer:

```bash
# Install Gazebo (Ignition Fortress)
sudo apt update
sudo apt install ros-humble-ros-gz

# Alternative: Install Ignition Gazebo directly
sudo apt install ignition-fortress
```

### Basic Gazebo Launch

```bash
# Launch Gazebo GUI
ign gazebo

# Launch without GUI (headless)
ign gazebo -s

# Launch with a specific world
ign gazebo -r simple_diff_drive.sdf
```

## Gazebo and ROS 2 Integration

### Gazebo ROS2 Packages

The `ros-gz` package provides bridges between Gazebo and ROS 2:

- `ros_gz_bridge`: Bidirectional communication between ROS 2 and Gazebo transport
- `ros_gz_image`: Bridge for image sensor data
- `ros_gz_point_cloud`: Point cloud generation from depth cameras
- `ros_gz_sim`: Integration between ROS 2 and Gazebo simulation

### Basic Integration Example

```python
# Example: Simple Gazebo-ROS2 interface
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from gazebo_msgs.srv import SpawnEntity

class GazeboRobotInterface(Node):
    def __init__(self):
        super().__init__('gazebo_robot_interface')
        
        # Publishers for controlling the robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers for sensor data
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def laser_callback(self, msg):
        self.get_logger().info(f'Laser scan: {len(msg.ranges)} readings')
        
    def camera_callback(self, msg):
        self.get_logger().info(f'Camera: {msg.width}x{msg.height} image received')
        
    def control_loop(self):
        # Implement control logic here
        cmd = Twist()
        cmd.linear.x = 0.5  # Move forward
        cmd.angular.z = 0.1  # Turn slightly
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboRobotInterface()
    
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

## Creating Robot Models for Gazebo

### URDF to SDF Conversion

Gazebo uses SDF (Simulation Description Format) but can read URDF files. For humanoid robots, your URDF needs to include Gazebo-specific elements:

```xml
<!-- Example humanoid robot URDF with Gazebo elements -->
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Links definition -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.6"/>
      </geometry>
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
  </joint>
  
  <link name="left_leg">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
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
  
  <!-- Gazebo-specific elements -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  
  <gazebo reference="left_leg">
    <material>Gazebo/Orange</material>
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
  
  <!-- Gazebo plugin for ROS2 control -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find my_robot_description)/config/my_robot_controllers.yaml</parameters>
    </plugin>
  </gazebo>
</robot>
```

### Gazebo Plugins for Humanoid Robots

For humanoid robots, you'll commonly use these plugins:

```xml
<!-- ros2_control plugin for actuator control -->
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find humanoid_description)/config/humanoid_controllers.yaml</parameters>
  </plugin>
</gazebo>

<!-- IMU sensor plugin -->
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>
    <topic>__default_topic__</topic>
    <ignition_frame_id>imu_link</ignition_frame_id>
  </sensor>
</gazebo>

<!-- Camera sensor plugin -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

## Physics Simulation Configuration

### Physics Engine Settings

Gazebo supports multiple physics engines (ODE, Bullet, SimBody). For humanoid robots, the configuration is critical:

```xml
<!-- physics.world (SDF format) -->
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
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

### Mass and Inertia Considerations

For realistic humanoid simulation, mass distribution is crucial:

```xml
<!-- Example mass and inertia values for humanoid components -->
<link name="torso">
  <inertial>
    <mass value="15.0"/>
    <!-- Calculated inertia tensor for a torso-like box -->
    <inertia ixx="0.8" ixy="0.0" ixz="0.0" iyy="0.6" iyz="0.0" izz="0.3"/>
  </inertial>
</link>

<link name="upper_arm">
  <inertial>
    <mass value="2.5"/>
    <!-- Approximate inertia for a cylindrical arm -->
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.01"/>
  </inertial>
</link>

<link name="lower_leg">
  <inertial>
    <mass value="3.0"/>
    <inertia ixx="0.3" ixy="0.0" ixz="0.0" iyy="0.3" iyz="0.0" izz="0.02"/>
  </inertial>
</link>
```

## Sensor Simulation in Gazebo

### LiDAR Simulation

For humanoid robots that need navigation and mapping capabilities:

```xml
<!-- 3D LiDAR sensor for humanoid robot -->
<gazebo reference="lidar_link">
  <sensor name="lidar_3d" type="ray">
    <ray>
      <scan>
        <horizontal>
          <samples>1080</samples>
          <resolution>1.0</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>64</samples>
          <resolution>1.0</resolution>
          <min_angle>-0.5236</min_angle>
          <max_angle>0.5236</max_angle>
        </vertical>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <always_on>true</always_on>
    <update_rate>10</update_rate>
    <visualize>false</visualize>
  </sensor>
</gazebo>
```

### IMU Simulation

Critical for humanoid balance and pose estimation:

```xml
<!-- IMU sensor for balance control -->
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>400</update_rate>
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

### Force/Torque Sensors

Essential for humanoid locomotion and manipulation:

```xml
<!-- Force/Torque sensor in foot for balance control -->
<gazebo>
  <sensor name="left_foot_ft_sensor" type="force_torque">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <force_torque>
      <frame>child</frame>
      <measure_direction>child_to_parent</measure_direction>
    </force_torque>
  </sensor>
</gazebo>
```

## Advanced Simulation Scenarios

### Multi-Robot Simulation

For training humanoid robots to interact with others:

```xml
<!-- Example launch file for multi-robot simulation -->
# launch/multi_humanoid_simulation.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch first humanoid
    robot1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('humanoid_bringup'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': 'humanoid1',
            'x_pose': '0.0',
            'y_pose': '0.5',
            'z_pose': '0.0'
        }.items()
    )
    
    # Launch second humanoid
    robot2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('humanoid_bringup'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': 'humanoid2',
            'x_pose': '2.0',
            'y_pose': '0.5',
            'z_pose': '0.0'
        }.items()
    )
    
    return LaunchDescription([
        robot1_launch,
        robot2_launch,
    ])
```

### Complex Environment Simulation

Creating realistic environments for humanoid testing:

```xml
<!-- Example world file with obstacles -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_test_world">
    <!-- Include outdoor environment -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Add obstacles -->
    <model name="table">
      <pose>2 0 0 0 0 0</pose>
      <link name="table_base">
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 0.8 0.8</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1.0 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>50.0</mass>
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
    
    <!-- Add stairs for locomotion testing -->
    <model name="stairs">
      <!-- Stair model definition -->
    </model>
    
    <!-- Physics configuration -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

## Simulation Validation and Accuracy

### Validating Simulation Results

To ensure your simulation accurately represents the real robot:

1. **Compare kinematic models**: Ensure URDF/SDF matches physical robot
2. **Validate sensor outputs**: Compare simulated vs. real sensor data
3. **Verify dynamics**: Check mass, inertia, and friction parameters
4. **Test control algorithms**: Validate that controllers work in simulation and reality

### Tuning Physics Parameters

Fine-tuning for accurate humanoid simulation:

```yaml
# Example parameter file for physics tuning
gazebo_model_config:
  physics:
    step_size: 0.001
    update_rate: 1000
    max_contacts: 20
    gravity: [0.0, 0.0, -9.81]
  
  joints:
    friction: 0.1
    damping: 0.5
    effort_limit: 100.0
  
  contacts:
    max_vel: 100.0
    min_depth: 0.001
    # Parameters for stable contact in humanoid joints
    kp: 1000000.0  # Stiffness parameter
    kd: 100.0      # Damping parameter
```

## Digital Twin Integration

### Bridging Simulation and Reality

Creating effective digital twins requires:

1. **Parameter synchronization**: Keep simulation parameters aligned with real robot
2. **Sensor fusion**: Combine simulated and real sensor data appropriately
3. **Model calibration**: Regularly update simulation parameters based on real robot performance

```python
# Example: Simulation-to-Reality parameter synchronization
class SimulationCalibrator(Node):
    def __init__(self):
        super().__init__('simulation_calibrator')
        
        # Service to update simulation parameters
        self.param_update_srv = self.create_service(
            UpdateSimParams,
            'update_simulation_params',
            self.update_params_callback
        )
        
        # Timer to periodically validate simulation accuracy
        self.validation_timer = self.create_timer(5.0, self.validate_simulation)
        
    def update_params_callback(self, request, response):
        # Update physics parameters in simulation
        self.update_inertial_params(request.inertial_values)
        self.update_joint_params(request.joint_values)
        self.update_friction_params(request.friction_values)
        
        response.success = True
        response.message = "Parameters updated successfully"
        return response
        
    def validate_simulation(self):
        # Compare simulation output with expected real-world behavior
        # This might involve comparing kinematic solutions, dynamic responses, etc.
        pass
```

## Performance Optimization

### Simulation Performance Tips

For real-time humanoid simulation:

1. **Reduce visual complexity**: Use simple geometric shapes instead of detailed meshes during computation-intensive tasks
2. **Optimize update rates**: Match physics update rates to control system requirements
3. **Simplify collision geometry**: Use simpler collision shapes where high precision isn't needed
4. **Control simulation complexity**: Limit the number of contacts and constraints in the simulation

## Summary

Gazebo provides a comprehensive simulation environment for humanoid robots that is essential for digital twin implementations. Proper setup of physics parameters, sensors, and environment models enables effective testing of humanoid control algorithms before deployment on physical hardware.

The integration with ROS 2 allows for seamless transition between simulated and real-world testing, making Gazebo a cornerstone of the humanoid robot development workflow.

## Exercises

1. Create a simple humanoid model in URDF and test it in Gazebo with basic joint control.

2. Configure IMU and force/torque sensors for a humanoid robot model and verify their output.

3. Create a world file with obstacles and test a humanoid robot's navigation capabilities in simulation.

## Further Reading

- Gazebo Documentation
- ROS 2 Gazebo Integration Guide
- "Simulation-Based Development for Robotics" by Kuffner et al.
- Gazebo Plugin Development Tutorials