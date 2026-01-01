---
title: Nodes, Topics, Services, Actions
sidebar_position: 4
description: Understanding the fundamental communication patterns in ROS 2 for humanoid robotics applications
---

# Nodes, Topics, Services, Actions

## Overview

Understanding the fundamental communication patterns in ROS 2 is crucial for creating effective digital twins of humanoid robots. This chapter covers the four primary communication mechanisms: nodes (the computing units), topics (publish/subscribe for continuous data), services (request/reply for synchronous operations), and actions (goal-oriented asynchronous communication for long-running tasks).

## Learning Objectives

By the end of this chapter, you should be able to:

- [ ] Create and manage ROS 2 nodes in Python
- [ ] Implement publisher-subscriber communication patterns
- [ ] Design and use service-based communication
- [ ] Understand when to use actions vs topics vs services
- [ ] Apply these patterns to humanoid robot simulation scenarios
- [ ] Configure appropriate Quality of Service (QoS) settings

## Nodes in ROS 2

Nodes are fundamental execution units that perform computations. In humanoid robots:
- Sensor nodes process data from cameras, LIDARs, IMUs
- Control nodes execute walking patterns or manipulation actions
- Perception nodes interpret sensory information

### Node Structure

```python
import rclpy
from rclpy.node import Node

class HumanoidRobotNode(Node):
    def __init__(self):
        super().__init__('humanoid_robot_node')
        # Initialize node components here
        self.get_logger().info('Humanoid Robot Node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidRobotNode()
    
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

### Node Parameters

Nodes can be configured with parameters for flexible behavior:

```python
class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Declare parameters with default values
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('control_frequency', 50)
        
        # Access parameter values
        self.max_velocity = self.get_parameter('max_velocity').value
        self.control_frequency = self.get_parameter('control_frequency').value
```

## Topics: Publisher-Subscriber Pattern

Topics enable asynchronous, one-way communication between nodes. They're ideal for continuous data streams like sensor readings.

### Publisher Implementation

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Publish at 50Hz
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Simulate joint positions
        self.joint_positions = [0.0] * 20  # 20 DOF humanoid
        
    def timer_callback(self):
        msg = JointState()
        msg.name = [f'joint_{i}' for i in range(20)]
        msg.position = self.joint_positions
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published joint states: {msg.position[:3]}...')

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    
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

### Subscriber Implementation

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)  # QoS depth
        self.subscription  # Prevent unused variable warning
        
    def listener_callback(self, msg):
        self.get_logger().info(f'Received joint positions: {msg.position[:3]}...')

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()
    
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

### Quality of Service (QoS) for Topics

Different types of data require different QoS settings:

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# For critical control commands
critical_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)

# For sensor data (e.g., cameras, LIDAR)
sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

# For logging/timing data
logging_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_ALL,
    depth=1000
)

# Publisher with specific QoS
self.publisher = self.create_publisher(JointState, 'joint_states', sensor_qos)
```

## Services: Request-Reply Pattern

Services enable synchronous communication where a client sends a request and waits for a response from a server.

### Service Server Implementation

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class RobotStateService(Node):
    def __init__(self):
        super().__init__('robot_state_service')
        self.srv = self.create_service(
            SetBool,
            'set_robot_state',
            self.set_robot_state_callback
        )
        
    def set_robot_state_callback(self, request, response):
        if request.data:
            self.get_logger().info('Activating robot')
            response.success = True
            response.message = 'Robot activated successfully'
        else:
            self.get_logger().info('Deactivating robot')
            response.success = True
            response.message = 'Robot deactivated successfully'
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotStateService()
    
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

### Service Client Implementation

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class RobotStateClient(Node):
    def __init__(self):
        super().__init__('robot_state_client')
        self.cli = self.create_client(SetBool, 'set_robot_state')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.request = SetBool.Request()

    def send_request(self, state):
        self.request.data = state
        self.future = self.cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = RobotStateClient()
    
    response = client.send_request(True)  # Activate robot
    print(f'Response: {response.message}')
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions: Goal-Oriented Communication

Actions are used for long-running tasks that may be preempted and provide feedback during execution.

### Action Server Implementation

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()
            
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')
        
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Result: {result.sequence}')
        
        return result

def main(args=None):
    rclpy.init(args=args)
    
    node = FibonacciActionServer()
    
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

### Action Client Implementation

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')

def main(args=None):
    rclpy.init(args=args)
    
    action_client = FibonacciActionClient()
    action_client.send_goal(10)  # Send a goal
    
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Communication Patterns in Humanoid Robotics

### Sensor Data Streams (Topics)

For continuous sensor data like joint positions, IMU readings, or camera images:

```python
# Example: IMU sensor publisher for humanoid robot
class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher = self.create_publisher(
            sensor_msgs.msg.Imu,
            '/humanoid/imu/data',
            10  # Use sensor QoS settings
        )
        
        # Publish at 100Hz for balance control
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        msg = sensor_msgs.msg.Imu()
        # Fill in IMU data simulation
        # This would be connected to actual sensor in real robot
        # or simulated sensor in digital twin
        self.publisher.publish(msg)
```

### Control Commands (Services)

For immediate robot control commands (e.g., enabling/disabling robot):

```python
# Example: Safe robot control service
class RobotControlService(Node):
    def __init__(self):
        super().__init__('robot_control_service')
        self.srv = self.create_service(
            SetBool,
            '/humanoid/safe_control',
            self.safe_control_callback
        )
        self.is_active = False
        
    def safe_control_callback(self, request, response):
        if self.is_safe_to_change_state(request.data):
            self.is_active = request.data
            response.success = True
            response.message = f"Robot {'enabled' if request.data else 'disabled'} successfully"
        else:
            response.success = False
            response.message = "Unsafe to change state at this time"
            
        return response
        
    def is_safe_to_change_state(self, new_state):
        # Implement safety checks here
        # Check if robot is in a safe position
        # Check if robot is currently executing critical tasks
        return True  # Simplified for example
```

### Complex Behaviors (Actions)

For complex, long-running behaviors like walking to a location:

```python
# Example: Navigation action for humanoid robot
class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')
        self._action_server = ActionServer(
            self,
            MoveBase,  # Custom action definition
            '/humanoid/navigate_to_pose',
            self.execute_callback)
        self.current_pose = None  # Robot's current pose

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing navigation goal...')
        
        # Perform path planning and navigation
        # This would involve coordinating multiple controllers
        # and monitoring the robot's state
        
        while not self.reached_goal(goal_handle.request.target_pose):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return MoveBase.Result()
            
            # Send feedback about progress
            feedback_msg = MoveBase.Feedback()
            feedback_msg.current_pose = self.current_pose
            goal_handle.publish_feedback(feedback_msg)
            
            # Sleep briefly to avoid busy waiting
            time.sleep(0.1)
        
        goal_handle.succeed()
        result = MoveBase.Result()
        result.success = True
        result.message = "Navigation completed successfully"
        
        return result
```

## Digital Twin Considerations

### Mirroring Physical Systems

When creating digital twins, communication patterns must mirror the physical robot:

- Use identical message types and structures
- Simulate sensor noise and delays appropriately
- Maintain timing consistency between simulation and reality

### Bridge Technologies

Digital twins often require bridging different systems:

- ROS 2 to Gazebo simulation
- ROS 2 to Unity visualization
- ROS 2 to real hardware (for hybrid systems)

## When to Use Each Pattern

### Use Topics When:
- You need asynchronous, continuous data flow (sensors, states)
- Multiple nodes need to receive the same data
- Real-time constraints allow for "best effort" delivery

### Use Services When:
- You need synchronous request/reply communication
- The operation is relatively quick (under 1 second)
- You need guaranteed response to specific requests

### Use Actions When:
- You need goal-oriented communication
- The operation will take a significant amount of time
- You need to provide feedback during execution
- The operation may be preempted by another goal

## Summary

Understanding the different communication patterns in ROS 2 is essential for effective humanoid robot development and digital twin creation. Each pattern serves specific purposes and choosing the right one for each interaction is key to building robust systems.

The node-topic-service-action paradigm provides a flexible foundation for creating complex humanoid robot systems that can operate effectively in both simulation and reality.

## Exercises

1. Create a ROS 2 node that publishes simulated sensor data (e.g., IMU, joint positions) and another that subscribes to this data and logs it.

2. Implement a service that validates robot poses for safety before allowing movement.

3. Design an action server that simulates a humanoid robot walking to a specific location with feedback on progress.

## Further Reading

- ROS 2 Communication Patterns Documentation
- Design Patterns for Robotic Systems
- "Programming Robots with ROS" by Morgan Quigley