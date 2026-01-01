---
title: Cognitive Planning - LLM → ROS 2 Actions
description: Implementing cognitive planning using Large Language Models to generate ROS 2 actions
sidebar_position: 3
---

# Cognitive Planning: LLM → ROS 2 Actions

## Introduction to Cognitive Planning

Cognitive planning bridges the gap between high-level natural language commands and low-level robot actions. Large Language Models (LLMs) excel at understanding semantic meaning and generating structured plans from natural language. In this section, we'll explore how to leverage LLMs to generate executable ROS 2 action sequences.

## Architecture Overview

The cognitive planning system consists of several components:

1. **Natural Language Input**: Commands from the user (e.g., "Pick up the red ball and place it in the box")
2. **LLM Planner**: Interprets the command and generates a sequence of atomic actions
3. **Action Executor**: Translates LLM-generated actions into ROS 2 service calls and action messages
4. **Feedback Loop**: Monitors execution and reports success/failure to the LLM

## Setting Up the LLM Interface

First, we'll create a node that interfaces with an LLM to generate action sequences:

```python
import rclpy
from rclpy.node import Node
import openai
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from action_msgs.msg import GoalStatus
import json

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')
        
        # Set your OpenAI API key (or use another LLM provider)
        # openai.api_key = "your-api-key-here"
        
        # Subscriber for high-level commands
        self.command_subscriber = self.create_subscription(
            String,
            'high_level_commands',
            self.command_callback,
            10
        )
        
        # Publisher for action sequences
        self.action_publisher = self.create_publisher(String, 'action_sequence', 10)
        
        # Publisher for status updates
        self.status_publisher = self.create_publisher(String, 'planning_status', 10)
        
    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received high-level command: {command}')
        
        # Publish status
        status_msg = String()
        status_msg.data = f'Processing command: {command}'
        self.status_publisher.publish(status_msg)
        
        # Generate action sequence using LLM
        try:
            action_sequence = self.generate_action_sequence(command)
            
            # Publish the action sequence
            action_msg = String()
            action_msg.data = json.dumps(action_sequence)
            self.action_publisher.publish(action_msg)
            
            self.get_logger().info(f'Generated action sequence: {action_sequence}')
            
        except Exception as e:
            self.get_logger().error(f'Error generating action sequence: {str(e)}')
            status_msg.data = f'Error: {str(e)}'
            self.status_publisher.publish(status_msg)
    
    def generate_action_sequence(self, command):
        # Define the context for the LLM
        context = """
        You are a robot task planner. Convert the human command into a sequence of robot actions.
        Each action should be one of the following types:
        - move_to: Move to a specific location
        - detect_object: Look for an object
        - grasp_object: Pick up an object
        - place_object: Place an object at a location
        - navigate_to: Navigate to a specific location in the environment
        - speak: Make the robot speak a phrase
        
        Respond with a JSON array of action objects, each with 'action_type' and 'parameters'.
        Example: [{"action_type": "move_to", "parameters": {"x": 1.0, "y": 2.0}}, {"action_type": "grasp_object", "parameters": {"object": "red ball"}}]
        """
        
        # Create the prompt for the LLM
        prompt = f"{context}\n\nHuman Command: {command}\n\nAction Sequence:"
        
        # Call the LLM (using OpenAI as an example)
        # In practice, you might use local models like Llama or other APIs
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",  # or "gpt-4" for better performance
            messages=[
                {"role": "system", "content": context},
                {"role": "user", "content": command}
            ],
            temperature=0.1,
            max_tokens=500
        )
        
        # Extract the action sequence from the response
        action_text = response.choices[0].message['content'].strip()
        
        # Try to parse as JSON, if not, try to extract JSON from the response
        try:
            action_sequence = json.loads(action_text)
        except json.JSONDecodeError:
            # Try to extract JSON from the response using regex
            import re
            json_match = re.search(r'\[.*\]', action_text, re.DOTALL)
            if json_match:
                action_sequence = json.loads(json_match.group())
            else:
                raise ValueError(f"Could not parse action sequence from LLM response: {action_text}")
        
        return action_sequence

def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()
    
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

## Action Execution Node

Next, we'll create a node that executes the action sequences generated by the LLM:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import json
import time

class ActionExecutorNode(Node):
    def __init__(self):
        super().__init__('action_executor_node')
        
        # Subscriber for action sequences
        self.action_subscriber = self.create_subscription(
            String,
            'action_sequence',
            self.action_callback,
            10
        )
        
        # Publishers for different robot actions
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, 'execution_status', 10)
        
        # Publisher for speech output
        self.speech_publisher = self.create_publisher(String, 'speech_commands', 10)
        
        # For this example, we'll simulate action execution
        # In a real robot, you would have clients for ROS 2 services/actions
        
    def action_callback(self, msg):
        try:
            action_sequence = json.loads(msg.data)
            self.get_logger().info(f'Executing action sequence: {action_sequence}')
            
            # Execute each action in the sequence
            for i, action in enumerate(action_sequence):
                self.get_logger().info(f'Executing action {i+1}/{len(action_sequence)}: {action}')
                
                # Publish status
                status_msg = String()
                status_msg.data = f'Executing: {action["action_type"]}'
                self.status_publisher.publish(status_msg)
                
                # Execute the action based on its type
                success = self.execute_action(action)
                
                if not success:
                    self.get_logger().error(f'Action failed: {action}')
                    status_msg.data = f'Action failed: {action["action_type"]}'
                    self.status_publisher.publish(status_msg)
                    return  # Stop execution on failure
                
                # Small delay between actions
                time.sleep(0.5)
            
            # All actions completed successfully
            status_msg = String()
            status_msg.data = 'All actions completed successfully'
            self.status_publisher.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error executing action sequence: {str(e)}')
    
    def execute_action(self, action):
        action_type = action['action_type']
        parameters = action.get('parameters', {})
        
        if action_type == 'move_to':
            return self.move_to(parameters)
        elif action_type == 'detect_object':
            return self.detect_object(parameters)
        elif action_type == 'grasp_object':
            return self.grasp_object(parameters)
        elif action_type == 'place_object':
            return self.place_object(parameters)
        elif action_type == 'navigate_to':
            return self.navigate_to(parameters)
        elif action_type == 'speak':
            return self.speak(parameters)
        else:
            self.get_logger().warn(f'Unknown action type: {action_type}')
            return False
    
    def move_to(self, params):
        # Simulate moving to a position
        x = params.get('x', 0.0)
        y = params.get('y', 0.0)
        
        self.get_logger().info(f'Moving to position ({x}, {y})')
        
        # In a real robot, you would send this to a navigation system
        # For simulation, we'll just publish a velocity command
        twist_msg = Twist()
        # Calculate direction to target
        # This is a simplified example - real navigation would use proper path planning
        twist_msg.linear.x = 0.5 if x > 0 else -0.5
        twist_msg.angular.z = 0.3 if y > 0 else -0.3
        self.cmd_vel_publisher.publish(twist_msg)
        
        # Simulate movement time
        time.sleep(2.0)
        
        # Stop the robot
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)
        
        return True
    
    def detect_object(self, params):
        # Simulate object detection
        object_name = params.get('object', 'unknown')
        self.get_logger().info(f'Detecting object: {object_name}')
        
        # In a real robot, you would call an object detection service
        # For simulation, we'll just return success
        time.sleep(1.0)
        
        # Publish status about detection
        status_msg = String()
        status_msg.data = f'Detected {object_name} at position (1.0, 2.0)'
        self.status_publisher.publish(status_msg)
        
        return True
    
    def grasp_object(self, params):
        # Simulate grasping an object
        object_name = params.get('object', 'unknown')
        self.get_logger().info(f'Grasping object: {object_name}')
        
        # In a real robot, you would call a grasping service
        # For simulation, we'll just return success
        time.sleep(1.5)
        
        return True
    
    def place_object(self, params):
        # Simulate placing an object
        location = params.get('location', 'unknown')
        self.get_logger().info(f'Placing object at: {location}')
        
        # In a real robot, you would call a placement service
        # For simulation, we'll just return success
        time.sleep(1.5)
        
        return True
    
    def navigate_to(self, params):
        # Simulate navigating to a location
        x = params.get('x', 0.0)
        y = params.get('y', 0.0)
        self.get_logger().info(f'Navigating to ({x}, {y})')
        
        # In a real robot, you would call the navigation service
        # For simulation, we'll just return success
        time.sleep(2.0)
        
        return True
    
    def speak(self, params):
        # Make the robot speak
        text = params.get('text', '')
        self.get_logger().info(f'Speaking: {text}')
        
        # Publish to speech system
        speech_msg = String()
        speech_msg.data = text
        self.speech_publisher.publish(speech_msg)
        
        return True

def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutorNode()
    
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

## Advanced Planning with Context Awareness

For more sophisticated planning, we need to consider the robot's current state and environment. Here's an enhanced version that incorporates context:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import openai
import json
import asyncio

class ContextAwarePlannerNode(Node):
    def __init__(self):
        super().__init__('context_aware_planner_node')
        
        # Set your OpenAI API key
        # openai.api_key = "your-api-key-here"
        
        # Subscribers
        self.command_subscriber = self.create_subscription(
            String,
            'high_level_commands',
            self.command_callback,
            10
        )
        
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            10
        )
        
        # Publisher for action sequences
        self.action_publisher = self.create_publisher(String, 'action_sequence', 10)
        
        # Publisher for status updates
        self.status_publisher = self.create_publisher(String, 'planning_status', 10)
        
        # Store robot state
        self.current_pose = None
        self.obstacles = []
        
    def laser_callback(self, msg):
        # Process laser scan to detect obstacles
        # This is a simplified example
        self.obstacles = []
        for i, range_val in enumerate(msg.ranges):
            if 0 < range_val < 1.0:  # Obstacle within 1 meter
                angle = msg.angle_min + i * msg.angle_increment
                # Convert polar to Cartesian coordinates
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                self.obstacles.append((x, y))
    
    def pose_callback(self, msg):
        # Store current robot pose
        self.current_pose = msg.pose
    
    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received high-level command: {command}')
        
        # Get current context
        context = self.get_current_context()
        
        # Generate action sequence using LLM with context
        try:
            action_sequence = self.generate_action_sequence_with_context(command, context)
            
            # Publish the action sequence
            action_msg = String()
            action_msg.data = json.dumps(action_sequence)
            self.action_publisher.publish(action_msg)
            
            self.get_logger().info(f'Generated action sequence with context: {action_sequence}')
            
        except Exception as e:
            self.get_logger().error(f'Error generating action sequence: {str(e)}')
    
    def get_current_context(self):
        # Create a context object with current robot state
        context = {
            'current_pose': {
                'x': self.current_pose.position.x if self.current_pose else 0,
                'y': self.current_pose.position.y if self.current_pose else 0,
                'z': self.current_pose.position.z if self.current_pose else 0,
                'orientation': {
                    'x': self.current_pose.orientation.x if self.current_pose else 0,
                    'y': self.current_pose.orientation.y if self.current_pose else 0,
                    'z': self.current_pose.orientation.z if self.current_pose else 0,
                    'w': self.current_pose.orientation.w if self.current_pose else 1
                }
            } if self.current_pose else None,
            'obstacles': [{'x': obs[0], 'y': obs[1]} for obs in self.obstacles[:10]]  # Limit to first 10 obstacles
        }
        
        return context
    
    def generate_action_sequence_with_context(self, command, context):
        # Create a more detailed prompt that includes context
        context_str = f"""
        Robot Context:
        - Current Position: {context['current_pose']}
        - Nearby Obstacles: {len(context['obstacles'])} detected
        
        Available Actions:
        - move_to: Move to a specific location
        - detect_object: Look for an object
        - grasp_object: Pick up an object
        - place_object: Place an object at a location
        - navigate_to: Navigate to a specific location in the environment
        - speak: Make the robot speak a phrase
        - avoid_obstacle: Move around an obstacle
        """
        
        prompt = f"""
        {context_str}
        
        Human Command: {command}
        
        Based on the current robot state and environment, generate a safe and effective sequence of actions.
        Consider obstacles when planning navigation actions.
        Respond with a JSON array of action objects, each with 'action_type' and 'parameters'.
        """
        
        # Call the LLM
        response = openai.ChatCompletion.create(
            model="gpt-4",  # Using GPT-4 for better reasoning
            messages=[
                {"role": "system", "content": context_str},
                {"role": "user", "content": command}
            ],
            temperature=0.1,
            max_tokens=500
        )
        
        # Extract the action sequence from the response
        action_text = response.choices[0].message['content'].strip()
        
        # Try to parse as JSON
        try:
            action_sequence = json.loads(action_text)
        except json.JSONDecodeError:
            import re
            json_match = re.search(r'\[.*\]', action_text, re.DOTALL)
            if json_match:
                action_sequence = json.loads(json_match.group())
            else:
                raise ValueError(f"Could not parse action sequence from LLM response: {action_text}")
        
        return action_sequence

def main(args=None):
    rclpy.init(args=args)
    node = ContextAwarePlannerNode()
    
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

## Integration with ROS 2 Action Architecture

For robust execution, we should use ROS 2's action architecture rather than simple publishers/subscribers. Here's how to integrate with ROS 2 actions:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
import json

# Assuming we have a custom action definition like MoveTo.action
# with fields: float64 x, float64 y, float64 theta
# and goal, result, feedback messages

class ActionBasedExecutorNode(Node):
    def __init__(self):
        super().__init__('action_based_executor_node')
        
        # Subscriber for action sequences
        self.action_subscriber = self.create_subscription(
            String,
            'action_sequence',
            self.action_callback,
            10
        )
        
        # Create action clients for different robot capabilities
        self.move_to_client = ActionClient(self, MoveTo, 'move_to')
        self.grasp_client = ActionClient(self, Grasp, 'grasp_object')
        self.navigate_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Wait for action servers to be available
        self.move_to_client.wait_for_server()
        self.grasp_client.wait_for_server()
        self.navigate_client.wait_for_server()
        
    def action_callback(self, msg):
        try:
            action_sequence = json.loads(msg.data)
            self.get_logger().info(f'Executing action sequence: {action_sequence}')
            
            # Execute each action in the sequence
            for i, action in enumerate(action_sequence):
                self.get_logger().info(f'Executing action {i+1}/{len(action_sequence)}: {action}')
                
                # Execute the action based on its type
                success = self.execute_action(action)
                
                if not success:
                    self.get_logger().error(f'Action failed: {action}')
                    return  # Stop execution on failure
                
        except Exception as e:
            self.get_logger().error(f'Error executing action sequence: {str(e)}')
    
    async def execute_action(self, action):
        action_type = action['action_type']
        parameters = action.get('parameters', {})
        
        if action_type == 'move_to':
            goal_msg = MoveTo.Goal()
            goal_msg.x = parameters.get('x', 0.0)
            goal_msg.y = parameters.get('y', 0.0)
            goal_msg.theta = parameters.get('theta', 0.0)
            
            # Send goal and wait for result
            future = self.move_to_client.send_goal_async(goal_msg)
            goal_handle = await future
            
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected')
                return False
            
            result_future = goal_handle.get_result_async()
            result = await result_future
            return result.result.success
        
        elif action_type == 'grasp_object':
            goal_msg = Grasp.Goal()
            goal_msg.object_id = parameters.get('object_id', '')
            goal_msg.pregrasp_position.x = parameters.get('pregrasp_x', 0.0)
            goal_msg.pregrasp_position.y = parameters.get('pregrasp_y', 0.0)
            goal_msg.pregrasp_position.z = parameters.get('pregrasp_z', 0.0)
            
            # Send goal and wait for result
            future = self.grasp_client.send_goal_async(goal_msg)
            goal_handle = await future
            
            if not goal_handle.accepted:
                self.get_logger().error('Grasp goal rejected')
                return False
            
            result_future = goal_handle.get_result_async()
            result = await result_future
            return result.result.success
        
        # Add other action types as needed
        else:
            self.get_logger().warn(f'Unknown action type: {action_type}')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = ActionBasedExecutorNode()
    
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

## Conclusion

Cognitive planning with LLMs enables humanoid robots to understand complex natural language commands and execute them through ROS 2. By combining LLMs with ROS 2's action architecture, we create a powerful system that can interpret high-level goals and translate them into precise robot behaviors. The next section will explore how to implement humanoid kinematics and dynamics to execute these planned actions effectively.