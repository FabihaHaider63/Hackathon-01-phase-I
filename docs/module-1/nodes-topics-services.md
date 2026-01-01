---
title: ROS 2 Nodes, Topics, and Services
sidebar_position: 2
description: Understanding the fundamental communication patterns in ROS 2 for humanoid robotics applications
---

# ROS 2 Nodes, Topics, and Services

## Overview

This chapter delves into the three fundamental communication patterns in ROS 2: nodes, topics, and services. Understanding these concepts is essential for building distributed robotic systems, particularly in humanoid robotics where multiple subsystems need to communicate seamlessly.

In humanoid robots, different components—such as perception, planning, and control systems—need to exchange information efficiently. Nodes, topics, and services provide the infrastructure for this communication while maintaining modularity and scalability.

## Learning Objectives

By the end of this chapter, you should be able to:

- [ ] Create and manage ROS 2 nodes in Python
- [ ] Implement publisher and subscriber patterns using topics
- [ ] Design and use service-based communication
- [ ] Apply appropriate communication patterns for different use cases in humanoid robotics

## ROS 2 Nodes

### What is a Node?

A node is a process that performs computation in a ROS 2 system. In humanoid robotics, nodes represent functional units:

- **Sensor Nodes**: Process data from cameras, LIDARs, or IMUs
- **Controller Nodes**: Generate control commands for actuators
- **AI Decision Nodes**: Make high-level decisions based on sensor data
- **Visualization Nodes**: Display robot state or sensor data

### Creating a Node in Python

Here's how to create a basic ROS 2 node using Python:

```python
import rclpy
from rclpy.node import Node

class BasicHumanoidNode(Node):
    def __init__(self):
        # Initialize the node with a name
        super().__init__('basic_humanoid_node')
        self.get_logger().info('Basic Humanoid Node has been started')

def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)
    
    # Create the node
    node = BasicHumanoidNode()
    
    # Keep the node alive
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Parameters

Nodes can be configured using parameters, which can be changed without recompilation:

```python
import rclpy
from rclpy.node import Node

class ConfigurableHumanoidNode(Node):
    def __init__(self):
        super().__init__('configurable_humanoid_node')
        
        # Declare parameters with default values
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('robot_name', 'humanoid')
        
        # Get parameter values
        self.max_speed = self.get_parameter('max_speed').value
        self.robot_name = self.get_parameter('robot_name').value
        
        self.get_logger().info(f'Robot: {self.robot_name}, Max Speed: {self.max_speed}')

def main(args=None):
    rclpy.init(args=args)
    node = ConfigurableHumanoidNode()
    
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

## Topics: Publisher-Subscriber Pattern

### When to Use Topics

Topics are ideal for:

- **Sensor Data Streams**: Camera images, LIDAR scans, IMU readings
- **Continuous Control Commands**: Joint positions, velocities, forces
- **State Information**: Robot pose, battery level, system status

Topics use a publish-subscribe pattern where publishers send data to a topic and subscribers receive the data. The communication is asynchronous and unidirectional.

### Publishing to Topics

Here's how to create a publisher that publishes joint positions for a humanoid robot:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        
        # Create a publisher for joint commands
        self.publisher = self.create_publisher(
            Float64MultiArray, 
            'joint_commands', 
            10
        )
        
        # Timer to publish messages at regular intervals
        self.timer = self.create_timer(0.1, self.publish_joint_commands)
        self.i = 0

    def publish_joint_commands(self):
        # Create the message
        msg = Float64MultiArray()
        
        # Set joint positions (for example, 6 joints)
        # This could be calculated based on desired pose
        joint_positions = [i * 0.1 for i in range(6)]
        msg.data = joint_positions
        
        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing joint commands: {msg.data}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    publisher = JointCommandPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscribing to Topics

Here's how to create a subscriber that listens to sensor data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class SensorDataProcessor(Node):
    def __init__(self):
        super().__init__('sensor_data_processor')
        
        # Create a subscription to joint states
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # Make sure the subscription is not kept alive after exit
        self.subscription  # prevent unused variable warning

    def joint_state_callback(self, msg):
        # Process the joint state message
        positions = np.array(msg.position)
        velocities = np.array(msg.velocity)
        efforts = np.array(msg.effort)
        
        self.get_logger().info(f'Received joint positions: {positions[:3]}')  # First 3 joints

def main(args=None):
    rclpy.init(args=args)
    processor = SensorDataProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services: Request-Response Pattern

### When to Use Services

Services are appropriate for:

- **Configuration Changes**: Setting parameters or modes
- **One-Time Requests**: Calibration, initialization, or state queries
- **Synchronous Operations**: When you need a guaranteed response

Services use a request-response pattern where clients send a request and receive a response.

### Creating a Service Server

Here's an example of a service that calculates a walking gait for a humanoid robot:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
import time

class GaitCalculationService(Node):
    def __init__(self):
        super().__init__('gait_calculation_service')
        
        # Create a service server
        self.srv = self.create_service(
            Trigger, 
            'calculate_gait', 
            self.calculate_gait_callback
        )

    def calculate_gait_callback(self, request, response):
        self.get_logger().info('Calculating gait...')
        
        # Simulate gait calculation (in a real system, this would be more complex)
        time.sleep(1)  # Simulating computation time
        
        # For this example, always succeed
        response.success = True
        response.message = 'Gait calculated successfully'
        
        return response

def main(args=None):
    rclpy.init(args=args)
    service = GaitCalculationService()
    
    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        pass
    finally:
        service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Service Client

Here's how to call the service from a client:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger

class GaitRequester(Node):
    def __init__(self):
        super().__init__('gait_requester')
        
        # Create a client for the calculate_gait service
        self.client = self.create_client(Trigger, 'calculate_gait')
        
        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.request = Trigger.Request()

    def send_request(self):
        # Send the request asynchronously
        self.future = self.client.call_async(self.request)
        return self.future

def main(args=None):
    rclpy.init(args=args)
    requester = GaitRequester()
    
    # Send the request
    future = requester.send_request()
    
    # Wait for the response
    rclpy.spin_until_future_complete(requester, future)
    
    # Process the response
    try:
        response = future.result()
        if response.success:
            print(f'Success: {response.message}')
        else:
            print(f'Failed: {response.message}')
    except Exception as e:
        requester.get_logger().info(f'Service call failed: {e}')
    finally:
        requester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quality of Service (QoS) Settings

For humanoid robotics applications, QoS settings are crucial to ensure reliable communication:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image

class QoSHumanoidNode(Node):
    def __init__(self):
        super().__init__('qos_humanoid_node')
        
        # Define QoS profile for sensor data (high reliability)
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Define QoS profile for control commands (real-time)
        control_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Create publishers with different QoS profiles
        self.sensor_pub = self.create_publisher(Image, 'sensor_data', sensor_qos)
        self.control_pub = self.create_publisher(Float64MultiArray, 'control_commands', control_qos)

def main(args=None):
    rclpy.init(args=args)
    node = QoSHumanoidNode()
    
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

## Design Patterns for Humanoid Robotics

### The Sensor-Controller-Executor Pattern

A common pattern in humanoid robotics is to separate sensing, decision-making, and execution:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.subscription = self.create_subscription(
            JointState, 'joint_states', self.state_callback, 10)
        self.publisher = self.create_publisher(
            Float64MultiArray, 'control_commands', 10)
    
    def state_callback(self, msg):
        # Calculate control commands based on state
        commands = self.calculate_commands(msg)
        self.publisher.publish(commands)
    
    def calculate_commands(self, state_msg):
        # Implementation of control algorithm
        commands = Float64MultiArray()
        # Compute joint commands based on the current state
        commands.data = [0.0] * len(state_msg.position)  # Placeholder
        return commands

class ExecutorNode(Node):
    def __init__(self):
        super().__init__('executor_node')
        self.subscription = self.create_subscription(
            Float64MultiArray, 'control_commands', self.command_callback, 10)
    
    def command_callback(self, msg):
        # Execute the control commands on the hardware
        self.execute_commands(msg.data)
    
    def execute_commands(self, commands):
        # Send commands to the actual robot hardware
        self.get_logger().info(f'Executing commands: {commands}')
```

## Summary

Nodes, topics, and services form the foundation of communication in ROS 2:

- **Nodes** are the basic units of computation in a ROS 2 system
- **Topics** enable asynchronous, one-way communication via publish-subscribe
- **Services** provide synchronous request-response communication

For humanoid robotics applications, these patterns enable the construction of modular, distributed systems where different components can be developed and tested independently.

## Exercises

1. Implement a node that publishes simulated IMU data for a humanoid robot.
2. Create a subscriber that listens to IMU data and calculates the robot's balance state.
3. Design a service that takes a desired walking pattern and returns the corresponding joint trajectories.
4. Research how Quality of Service settings affect real-time performance in ROS 2 and explain why they are important for humanoid robotics.

## Further Reading

- ROS 2 Design: https://design.ros2.org/
- Quality of Service in ROS 2: https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html
- "Programming Robots with ROS" by Quigley et al.