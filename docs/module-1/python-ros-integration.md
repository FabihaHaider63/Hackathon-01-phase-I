---
title: Bridging Python AI Agents with ROS Controllers
sidebar_position: 3
description: Integrating Python-based AI agents with ROS 2 control systems for humanoid robotics
---

# Bridging Python AI Agents with ROS Controllers

## Overview

This chapter explores how to connect Python-based AI agents with ROS 2 control systems, which is fundamental for creating intelligent humanoid robots. We'll cover the practical aspects of integrating AI decision-making with robotic control using the `rclpy` library.

The bridge between AI and control systems is crucial for humanoid robots that need to make intelligent decisions based on sensor data and execute complex behaviors. This chapter demonstrates how Python's rich AI ecosystem (including libraries like PyTorch, TensorFlow, and scikit-learn) can interact seamlessly with ROS 2's control infrastructure.

## Learning Objectives

By the end of this chapter, you should be able to:

- [ ] Integrate Python AI agents with ROS 2 control systems using rclpy
- [ ] Design communication patterns between AI and control nodes
- [ ] Implement decision-making algorithms that interact with ROS messages
- [ ] Handle real-time requirements when connecting AI to physical systems

## Introduction to rclpy

`rclpy` is the Python client library for ROS 2, allowing Python programs to interface with the ROS 2 middleware. It enables Python nodes to publish, subscribe, provide services, and call services.

### Installing rclpy

`rclpy` is typically included with a standard ROS 2 installation. To verify:

```bash
pip list | grep rclpy
```

### Basic rclpy Structure

Every rclpy-based node follows a common structure:

```python
import rclpy
from rclpy.node import Node

class AIControlNode(Node):
    def __init__(self):
        super().__init__('ai_control_node')
        # Initialize publishers, subscribers, timers, etc.

def main(args=None):
    rclpy.init(args=args)
    node = AIControlNode()
    
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

## Example: Simple AI Decision Maker

Let's start with a simple example that demonstrates a Python AI agent making decisions based on sensor input:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class SimpleAIDecisionMaker(Node):
    def __init__(self):
        super().__init__('simple_ai_decision_maker')
        
        # Subscribe to laser scan data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        # Publisher for velocity commands
        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Robot state
        self.obstacle_ahead = False
        
    def scan_callback(self, msg):
        # Simple AI: detect obstacles in front of the robot
        # This is a very basic example - real AI would be more sophisticated
        if len(msg.ranges) > 0:
            # Look at the front 30 degrees of the scan
            front_ranges = msg.ranges[:len(msg.ranges)//12] + msg.ranges[-len(msg.ranges)//12:]
            
            # Filter out invalid ranges
            valid_ranges = [r for r in front_ranges if r != float('inf') and not np.isnan(r)]
            
            if valid_ranges and min(valid_ranges) < 0.5:  # Obstacle within 0.5m
                self.obstacle_ahead = True
            else:
                self.obstacle_ahead = False
                
            # Make a decision and publish command
            self.make_decision()
    
    def make_decision(self):
        cmd_msg = Twist()
        
        if self.obstacle_ahead:
            # Stop and turn right
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = -0.5  # Turn right
        else:
            # Move forward
            cmd_msg.linear.x = 0.3
            cmd_msg.angular.z = 0.0
            
        self.publisher.publish(cmd_msg)
        self.get_logger().info(f'Published: linear.x={cmd_msg.linear.x}, angular.z={cmd_msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    ai_node = SimpleAIDecisionMaker()
    
    try:
        rclpy.spin(ai_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced AI Integration

For more sophisticated AI agents, we'll look at how to integrate machine learning models with ROS 2.

### Using TensorFlow/Keras with ROS 2

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import tensorflow as tf
import numpy as np

class TFAIDecisionMaker(Node):
    def __init__(self):
        super().__init__('tf_ai_decision_maker')
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Load trained model (assuming model is saved)
        # self.model = tf.keras.models.load_model('path/to/your/model')
        
        # For this example, create a dummy model
        self.model = self.create_dummy_model()
        
        # Subscribe to camera image
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher for velocity commands
        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
    def create_dummy_model(self):
        # Create a simple dummy model for demonstration
        model = tf.keras.Sequential([
            tf.keras.layers.Dense(64, activation='relu', input_shape=(10,)),
            tf.keras.layers.Dense(32, activation='relu'),
            tf.keras.layers.Dense(2, activation='linear')  # Output: [linear, angular]
        ])
        return model
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Preprocess image for the model (resize, normalize, etc.)
            processed_image = self.preprocess_image(cv_image)
            
            # Get AI decision
            decision = self.model.predict(processed_image)
            
            # Create and publish Twist command
            cmd_msg = Twist()
            cmd_msg.linear.x = float(decision[0][0])
            cmd_msg.angular.z = float(decision[0][1])
            
            self.publisher.publish(cmd_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def preprocess_image(self, image):
        # Resize image to model input size (e.g., 224x224)
        resized = cv2.resize(image, (100, 100))
        
        # Normalize pixel values
        normalized = resized.astype(np.float32) / 255.0
        
        # Flatten or reshape as needed by your model
        reshaped = np.expand_dims(normalized, axis=0)  # Add batch dimension
        
        return reshaped

def main(args=None):
    rclpy.init(args=args)
    ai_node = TFAIDecisionMaker()
    
    try:
        rclpy.spin(ai_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Design Patterns for AI-ROS Integration

### 1. State Machine with AI Decision Engine

This pattern combines traditional state machines with AI decision-making:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from enum import Enum

class RobotState(Enum):
    IDLE = 1
    NAVIGATING = 2
    AVOIDING_OBSTACLE = 3
    PICKING_UP_OBJECT = 4

class AIBehaviorEngine(Node):
    def __init__(self):
        super().__init__('ai_behavior_engine')
        
        # Subscriptions for sensing
        self.scan_subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.image_subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        
        # Publisher for commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize state
        self.current_state = RobotState.IDLE
        self.scan_data = None
        self.image_data = None
        
        # Timer for state transitions
        self.timer = self.create_timer(0.1, self.state_machine)
        
    def scan_callback(self, msg):
        self.scan_data = msg
    
    def image_callback(self, msg):
        self.image_data = msg
    
    def state_machine(self):
        # AI-based state transition logic
        next_state = self.determine_next_state()
        
        if next_state != self.current_state:
            self.get_logger().info(f'State transition: {self.current_state} -> {next_state}')
            self.current_state = next_state
        
        # Execute behavior for current state
        cmd = self.execute_behavior()
        self.cmd_publisher.publish(cmd)
    
    def determine_next_state(self):
        # Simple AI decision logic (in practice, this would be more complex)
        if self.scan_data and min(self.scan_data.ranges) < 0.5:
            return RobotState.AVOIDING_OBSTACLE
        elif self.image_data:  # If we have image data, maybe there's an object to pick up
            # This would involve more complex image processing
            return RobotState.NAVIGATING
        else:
            return RobotState.IDLE
    
    def execute_behavior(self):
        cmd = Twist()
        
        if self.current_state == RobotState.NAVIGATING:
            cmd.linear.x = 0.5
        elif self.current_state == RobotState.AVOIDING_OBSTACLE:
            cmd.angular.z = 1.0
        elif self.current_state == RobotState.IDLE:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        return cmd

def main(args=None):
    rclpy.init(args=args)
    behavior_node = AIBehaviorEngine()
    
    try:
        rclpy.spin(behavior_node)
    except KeyboardInterrupt:
        pass
    finally:
        behavior_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Behavior Tree with ROS Integration

Behavior trees are useful for more complex decision-making:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class BehaviorNode:
    def __init__(self):
        self.status = "RUNNING"  # "RUNNING", "SUCCESS", "FAILURE"
    
    def tick(self, sensor_data):
        raise NotImplementedError("Subclasses must implement tick")
    
    def reset(self):
        self.status = "RUNNING"

class SequenceNode(BehaviorNode):
    def __init__(self, children):
        super().__init__()
        self.children = children
        self.current_child_idx = 0
    
    def tick(self, sensor_data):
        while self.current_child_idx < len(self.children):
            child = self.children[self.current_child_idx]
            child_status = child.tick(sensor_data)
            
            if child_status == "RUNNING":
                return "RUNNING"
            elif child_status == "FAILURE":
                self.current_child_idx = 0
                self.status = "FAILURE"
                return "FAILURE"
            else:  # SUCCESS
                self.current_child_idx += 1
        
        if self.current_child_idx >= len(self.children):
            self.current_child_idx = 0
            self.status = "SUCCESS"
            return "SUCCESS"

class HasPathNode(BehaviorNode):
    def tick(self, sensor_data):
        # In a real implementation, this would check for a valid path
        has_path = True  # Placeholder
        if has_path:
            self.status = "SUCCESS"
            return "SUCCESS"
        else:
            self.status = "FAILURE"
            return "FAILURE"

class MoveToGoalNode(BehaviorNode):
    def __init__(self, cmd_publisher):
        super().__init__()
        self.cmd_publisher = cmd_publisher
        self.cmd_msg = Twist()
    
    def tick(self, sensor_data):
        # In a real implementation, this would move toward a goal
        self.cmd_msg.linear.x = 0.5  # Move forward
        self.cmd_publisher.publish(self.cmd_msg)
        self.status = "RUNNING"
        return "RUNNING"  # RUNNING because this is a continuous action

class AIBehaviorTree(Node):
    def __init__(self):
        super().__init__('ai_behavior_tree')
        
        # Subscription
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
        # Publisher
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize behavior tree
        self.scan_data = None
        self.root = SequenceNode([
            HasPathNode(),
            MoveToGoalNode(self.cmd_publisher)
        ])
        
        # Timer to run the behavior tree
        self.timer = self.create_timer(0.1, self.run_behavior_tree)
    
    def scan_callback(self, msg):
        self.scan_data = msg
    
    def run_behavior_tree(self):
        if self.scan_data:
            # Execute behavior tree
            status = self.root.tick(self.scan_data)
            self.get_logger().info(f'Behavior tree status: {status}')

def main(args=None):
    rclpy.init(args=args)
    bt_node = AIBehaviorTree()
    
    try:
        rclpy.spin(bt_node)
    except KeyboardInterrupt:
        pass
    finally:
        bt_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Considerations

When bridging AI agents with ROS controllers, performance is critical, especially for real-time applications:

### 1. Threading and Concurrency

```python
import rclpy
from rclpy.node import Node
from threading import Thread
import queue
import time

class ThreadingAIAgent(Node):
    def __init__(self):
        super().__init__('threading_ai_agent')
        
        # Queue for receiving sensor data in a separate thread
        self.data_queue = queue.Queue()
        
        # Start AI processing thread
        self.ai_thread = Thread(target=self.ai_process_loop)
        self.ai_thread.start()
        
        # Publisher
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
    
    def ai_process_loop(self):
        while rclpy.ok():
            try:
                # Get data from queue (non-blocking)
                if not self.data_queue.empty():
                    sensor_data = self.data_queue.get_nowait()
                    
                    # Process with AI (potentially expensive)
                    command = self.ai_decision_process(sensor_data)
                    
                    # Publish result
                    self.cmd_publisher.publish(command)
            except queue.Empty:
                time.sleep(0.01)  # Small delay to prevent busy waiting
    
    def ai_decision_process(self, sensor_data):
        # Complex AI processing (e.g., neural network inference)
        cmd = Twist()
        cmd.linear.x = 1.0  # Example command
        return cmd
    
    def destroy_node(self):
        # Properly shut down threads
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    ai_node = ThreadingAIAgent()
    
    try:
        rclpy.spin(ai_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Model Optimization

For efficient AI inference on robots:

1. **Quantization**: Reduce model precision from 32-bit to 8-bit
2. **Pruning**: Remove less important neurons/connections
3. **Distillation**: Create smaller, faster student models
4. **Edge AI Frameworks**: Use specialized frameworks like OpenVINO, TensorRT, or ONNX Runtime

## Summary

Integrating Python AI agents with ROS controllers involves:

1. Using `rclpy` to create ROS nodes in Python
2. Defining appropriate message types for communication
3. Implementing design patterns like state machines or behavior trees
4. Considering performance aspects like threading and model optimization

This bridge enables humanoid robots to leverage Python's rich AI ecosystem while using ROS 2's proven robotics infrastructure.

## Exercises

1. Implement an AI agent that uses reinforcement learning to navigate a humanoid robot through a known environment.
2. Create a behavior tree that controls a humanoid robot to pick up and place objects in a specific sequence.
3. Design a threading architecture that allows a complex AI model to run alongside real-time control without blocking.
4. Research and implement a simple neural network that can be used for humanoid robot balance control.

## Further Reading

- ROS 2 with Python: https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Node.html
- Behavior Trees in Robotics: http://www.diag.uniroma1.it/~bloisi/didattica/robotica/AA2020-2021/Robotics-2021-07-behavior-trees.pdf
- Robotics and AI Integration: "Introduction to Autonomous Robots" by Nikolaus Correll