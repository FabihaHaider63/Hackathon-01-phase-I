---
title: ROS 2 Fundamentals (Weeks 3–5)
sidebar_position: 2
description: Core concepts of ROS 2 architecture, nodes, topics, services, and actions for humanoid robotics
---

# ROS 2 Fundamentals (Weeks 3–5)

## Overview

This chapter provides an introduction to the fundamental concepts of ROS 2, building on the introduction to the digital twin concept. ROS 2 (Robot Operating System 2) is the middleware that enables communication between different components of a robotic system, serving as the "nervous system" of humanoid robots. Understanding these fundamentals is crucial for creating effective digital twins that accurately reflect real-world behavior.

## Learning Objectives

By the end of this chapter, you should be able to:

- [ ] Understand the core concepts of ROS 2 architecture
- [ ] Differentiate between nodes, topics, services, and actions
- [ ] Create basic ROS 2 nodes in Python
- [ ] Implement publisher-subscriber and client-server communication patterns
- [ ] Configure and launch ROS 2 packages using launch files
- [ ] Apply these concepts to humanoid robot simulation scenarios

## Weekly Breakdown

This chapter covers material intended for Weeks 3-5 of the course:

- **Week 3**: ROS 2 architecture and design principles
- **Week 4**: Core concepts: nodes, topics, services, actions
- **Week 5**: Building ROS 2 packages with Python and launch files

## ROS 2 Architecture

ROS 2 addresses key limitations of the original ROS framework, particularly with real-time support, security, and multi-robot support. The architecture is built on:

### DDS (Data Distribution Service)

- Implements publish/subscribe communication pattern
- Provides Quality of Service (QoS) settings for reliable communication
- Supports real-time systems and multi-robot coordination

### Client Library

- Provides APIs in multiple languages (C++, Python, etc.)
- Handles communication with DDS implementation
- Manages node lifecycle and communication interfaces

### Composition

- Nodes can run in separate processes or be composed together in a single process
- Enables performance optimization based on communication requirements
- Facilitates efficient simulation scenarios

## Key Concepts Overview

### Nodes

Nodes are fundamental execution units that perform computations. In humanoid robots:
- Sensor nodes process data from cameras, LIDARs, IMUs
- Control nodes execute walking patterns or manipulation actions
- Perception nodes interpret sensory information

### Topics

Topics enable asynchronous data exchange between nodes:
- Used for continuous data streams like sensor data
- Publisher-subscriber pattern with publish frequency
- Message types define data structure (e.g., sensor_msgs/Image, geometry_msgs/Twist)

### Services

Services provide synchronous request/reply communication:
- Used for operations requiring specific responses
- Request-reply pattern with blocking calls
- Common for configuration, calibration, or one-time operations

### Actions

Actions support goal-oriented asynchronous communication:
- Used for long-running tasks with feedback
- Support preempting goals if needed
- Ideal for navigation, manipulation, or complex robot behaviors

## Setting Up ROS 2 Development Environment

Before creating nodes, you'll need to set up your environment:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash  # Replace 'humble' with your ROS 2 distribution

# Create a workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Creating Your First ROS 2 Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python humanoid_robot_examples
```

This creates a basic package structure with the necessary configuration files.

## Summary

Understanding the fundamentals of ROS 2 is crucial for creating effective digital twins of humanoid robots. The publish/subscribe, request/reply, and action communication patterns provide the flexibility needed to model complex robot behaviors in simulation.

## Exercises

1. Research the differences between ROS 1 and ROS 2, focusing on how these differences benefit humanoid robotics.
2. Design a simple ROS 2 system architecture for a basic humanoid robot with sensors and actuators.
3. Create a ROS 2 workspace and verify your development environment is properly set up.

## Further Reading

- ROS 2 Documentation: https://docs.ros.org/
- "Programming Robots with ROS" by Morgan Quigley, Brian Gerkey, and William Smart