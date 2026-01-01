---
title: ROS 2 Architecture
sidebar_position: 3
description: Understanding the architecture and design principles of ROS 2 for humanoid robotics applications
---

# ROS 2 Architecture

## Overview

ROS 2 (Robot Operating System 2) is not an operating system in the traditional sense, but rather a collection of software frameworks and tools that help developers create robotic applications. This chapter delves into the architectural design principles that make ROS 2 particularly suitable for humanoid robotics applications, especially in the context of digital twins and simulation.

## Learning Objectives

By the end of this chapter, you should be able to:

- [ ] Explain the key architectural differences between ROS 1 and ROS 2
- [ ] Identify the role of DDS in ROS 2 architecture
- [ ] Understand Quality of Service (QoS) settings and their impact on communication
- [ ] Recognize how ROS 2 architecture supports real-time systems and security
- [ ] Apply architectural patterns to humanoid robot simulation scenarios

## Architectural Evolution: ROS 1 to ROS 2

### ROS 1 Limitations

The original ROS framework had several architectural limitations that made it challenging for humanoid robotics:

- **Single-threaded execution model**: Could become a bottleneck for complex humanoid robots with multiple sensors
- **No security model**: Difficult to deploy robots in shared environments
- **No real-time support**: Critical for humanoid robot control systems
- **Master-based architecture**: Single point of failure
- **Multi-robot challenges**: Difficult to coordinate multiple robots

### ROS 2 Solutions

ROS 2 was designed to address these limitations with a new architecture based on DDS (Data Distribution Service).

## DDS (Data Distribution Service) Foundation

### What is DDS?

DDS is an open standard for distributed real-time systems that provides:

- **Publish/Subscribe Model**: Decentralized communication without a central master
- **Discovery Protocol**: Automatic discovery of available publishers and subscribers
- **Quality of Service Settings**: Configurable communication characteristics
- **Real-Time Support**: Designed for time-critical applications

### DDS in ROS 2

ROS 2 uses DDS as its underlying communication middleware, which means:

- No single point of failure
- Built-in support for multi-robot coordination
- Real-time communication capabilities
- Support for various DDS implementations (Fast DDS, Cyclone DDS, RTI Connext DDS)

### DDS Implementation Options

ROS 2 supports multiple DDS implementations:

- **Fast DDS (default)**: Open-source implementation by eProsima
- **Cyclone DDS**: Open-source implementation by Eclipse Foundation
- **RTI Connext DDS**: Commercial implementation with advanced features

## Quality of Service (QoS) Profiles

QoS profiles allow fine-tuning of communication characteristics:

### Reliability Policy

- **Reliable**: Guarantees message delivery (default for topics)
- **Best Effort**: No guarantee of delivery (suitable for sensor data)

### Durability Policy

- **Transient Local**: Publishers send recent messages to new subscribers
- **Volatile**: New subscribers only receive future messages

### History Policy

- **Keep Last**: Store only the most recent messages
- **Keep All**: Store all messages (use with caution)

### Example QoS Configuration

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# For critical control commands
critical_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)

# For sensor data (e.g., cameras)
sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)
```

## Node Architecture

### Node Lifecycle

In ROS 2, nodes have a well-defined lifecycle:

1. **Unconfigured**: Node exists but is not active
2. **Inactive**: Node is configured but not running
3. **Active**: Node is fully operational
4. **Finalized**: Node is shutting down

### Composition

ROS 2 supports composition, allowing multiple nodes to run in the same process:

```python
# Example of composition
from rclpy import init
from rclpy.executors import SingleThreadedExecutor
from my_nodes import SensorNode, ControllerNode

def main():
    init()
    
    executor = SingleThreadedExecutor()
    sensor_node = SensorNode()
    controller_node = ControllerNode()
    
    executor.add_node(sensor_node)
    executor.add_node(controller_node)
    
    try:
        executor.spin()
    finally:
        sensor_node.destroy_node()
        controller_node.destroy_node()
        executor.shutdown()
```

## Security Architecture

ROS 2 includes a comprehensive security framework:

### Authentication

- Verify the identity of nodes in the system
- Uses X.509 certificates for cryptographic authentication

### Access Control

- Define which nodes can communicate with each other
- Configure read/write permissions for topics and services

### Encryption

- Encrypt communication between nodes
- Protect sensitive data and commands

## Real-Time Support

ROS 2 provides real-time capabilities essential for humanoid robots:

### Schedulability

- Support for real-time scheduling policies (SCHED_FIFO, SCHED_RR)
- Memory pre-allocation to avoid dynamic allocation during critical sections

### Time Synchronization

- Support for different time sources
- Ability to coordinate with simulation time in digital twins

## Architecture for Humanoid Robotics

### Modular Design Philosophy

The ROS 2 architecture supports the modular design required for humanoid robotics:

- **Sensors**: Independent nodes for cameras, LIDARs, IMUs, force/torque sensors
- **Controllers**: Independent nodes for joint control, balance, gait generation
- **Perception**: Independent nodes for object detection, SLAM, scene understanding
- **Planning**: Independent nodes for motion planning, path planning, task planning

### Simulation Integration

ROS 2 architecture facilitates seamless integration with simulation:

- Same message types used in simulation and reality
- Bridge nodes to connect simulation and real systems
- Time synchronization between simulation and real world

## Architecture Best Practices

### Design Patterns

1. **Single Responsibility**: Each node should have a single, well-defined purpose
2. **Loose Coupling**: Minimize dependencies between nodes
3. **High Cohesion**: Group related functionality within the same node
4. **Interface Segregation**: Define clear interfaces between nodes

### Performance Considerations

- Use appropriate QoS settings for different message types
- Consider composition for nodes with high-frequency communication
- Use services for low-frequency, request-reply interactions
- Use topics for high-frequency, asynchronous communication
- Use actions for long-duration, goal-oriented behaviors

## Digital Twin Integration

The ROS 2 architecture is well-suited for digital twin implementations:

### Mirroring Physical Systems

- Same interface definitions in simulation and reality
- Shared message types and service definitions
- Consistent node structure between physical and virtual systems

### Bridge Technologies

- `ros_gz_bridge` for connecting ROS 2 to Gazebo
- `ros_unity_bridge` for connecting ROS 2 to Unity
- Standardized bridge protocols for consistent behavior

## Summary

The ROS 2 architecture provides a robust foundation for humanoid robotics applications. Its DDS-based communication, QoS settings, security features, and real-time support make it particularly suitable for the complex communication requirements of humanoid robots and their digital twins.

Understanding the architectural principles helps in designing effective systems that leverage the full capabilities of ROS 2 while addressing the unique challenges of humanoid robotics.

## Exercises

1. Research and compare the different DDS implementations supported by ROS 2, analyzing their performance characteristics and use cases.

2. Design a ROS 2 system architecture for a humanoid robot with 20+ joints, multiple sensors, and complex control systems.

3. Implement a simple multi-node ROS 2 system with different QoS profiles to understand their impact on communication.

## Further Reading

- "ROS 2 Design: The Next Generation ROS" by Tully Foote
- DDS Specification by Object Management Group
- "Real-Time Systems and the Robotics Operating System" by Morgan Quigley