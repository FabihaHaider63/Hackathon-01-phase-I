---
title: Manipulation & Grasping
description: Techniques for robotic manipulation and object grasping in humanoid robots
sidebar_position: 6
---

# Manipulation & Grasping

## Introduction to Robotic Manipulation

Robotic manipulation involves the controlled movement and interaction of a robot with objects in its environment. For humanoid robots, manipulation is particularly challenging due to the need to coordinate multiple degrees of freedom while maintaining balance. This section covers the fundamental concepts of robotic manipulation and grasping for humanoid systems.

## Kinematic Chains and Manipulator Arms

Humanoid robots typically have two main kinematic chains for manipulation: the arms. Each arm forms a kinematic chain from the shoulder to the hand, with multiple joints that can be controlled to achieve desired end-effector positions and orientations.

```python
import numpy as np
import math

def dh_transform(a, alpha, d, theta):
    """Calculate the DH transformation matrix"""
    return np.array([
        [math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
        [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
        [0, math.sin(alpha), math.cos(alpha), d],
        [0, 0, 0, 1]
    ])

class ManipulatorArm:
    def __init__(self, dh_params):
        """
        Initialize manipulator arm with DH parameters
        dh_params: List of tuples (a, alpha, d, theta) for each joint
        """
        self.dh_params = dh_params
        self.num_joints = len(dh_params)
    
    def forward_kinematics(self, joint_angles):
        """Calculate end-effector pose given joint angles"""
        if len(joint_angles) != self.num_joints:
            raise ValueError("Number of joint angles must match number of joints")
        
        # Start with identity transformation
        T = np.eye(4)
        
        # Apply each joint transformation
        for i in range(self.num_joints):
            a, alpha, d, _ = self.dh_params[i]
            theta = joint_angles[i]
            
            # Update theta with joint angle
            T_joint = dh_transform(a, alpha, d, theta)
            T = T @ T_joint
        
        return T
    
    def jacobian(self, joint_angles):
        """Calculate the geometric Jacobian matrix"""
        # Get the transformation matrices for each joint
        T_cum = np.eye(4)
        T_joints = [T_cum.copy()]  # Include base transform
        
        for i in range(self.num_joints):
            a, alpha, d, _ = self.dh_params[i]
            theta = joint_angles[i]
            
            T_joint = dh_transform(a, alpha, d, theta)
            T_cum = T_cum @ T_joint
            T_joints.append(T_cum.copy())
        
        # End-effector position
        T_end = T_joints[-1]
        end_pos = T_end[:3, 3]
        
        # Initialize Jacobian
        J = np.zeros((6, self.num_joints))  # 6 DOF (position + orientation)
        
        # Calculate each column of the Jacobian
        for i in range(self.num_joints):
            T_i = T_joints[i]
            z_i = T_i[:3, 2]  # z-axis of joint i
            r_i = T_i[:3, 3]  # position of joint i
            
            # Linear velocity component
            r_diff = end_pos - r_i
            J[:3, i] = np.cross(z_i, r_diff)
            
            # Angular velocity component
            J[3:, i] = z_i
        
        return J
```

## Grasp Planning

Grasp planning involves determining how to position the robot's hand to securely grasp an object. This includes selecting contact points, grasp type, and hand configuration.

```python
class GraspPlanner:
    def __init__(self, hand_model):
        self.hand_model = hand_model  # Model of the robot hand
    
    def plan_grasp(self, object_info):
        """
        Plan a grasp for the given object
        object_info: Dictionary containing object properties (shape, size, pose, etc.)
        """
        # Determine object properties
        obj_pose = object_info['pose']
        obj_shape = object_info['shape']
        obj_size = object_info['size']
        
        # Generate candidate grasp poses based on object shape
        candidate_grasps = self.generate_grasp_candidates(obj_shape, obj_pose, obj_size)
        
        # Evaluate each candidate grasp
        best_grasp = None
        best_score = -float('inf')
        
        for grasp in candidate_grasps:
            score = self.evaluate_grasp(grasp, object_info)
            if score > best_score:
                best_score = score
                best_grasp = grasp
        
        return best_grasp
    
    def generate_grasp_candidates(self, obj_shape, obj_pose, obj_size):
        """Generate potential grasp poses based on object shape"""
        candidates = []
        
        if obj_shape == 'cylinder':
            # Generate grasps around the cylinder
            radius = obj_size[0] / 2
            height = obj_size[1]
            
            # Top grasp
            top_grasp = {
                'position': np.array([obj_pose[0], obj_pose[1], obj_pose[2] + height/2 + 0.05]),
                'orientation': np.array([0, 0, 0, 1]),  # Quaternion for top-down grasp
                'type': 'top_grasp'
            }
            candidates.append(top_grasp)
            
            # Side grasps
            for angle in np.linspace(0, 2*np.pi, 8):
                side_grasp = {
                    'position': np.array([
                        obj_pose[0] + radius * np.cos(angle),
                        obj_pose[1] + radius * np.sin(angle),
                        obj_pose[2]
                    ]),
                    'orientation': np.array([0, 0, 0, 1]),  # Adjust as needed
                    'type': 'side_grasp'
                }
                candidates.append(side_grasp)
        
        elif obj_shape == 'box':
            # Generate corner and face grasps for a box
            width, depth, height = obj_size
            
            # Top face grasp
            top_grasp = {
                'position': np.array([obj_pose[0], obj_pose[1], obj_pose[2] + height/2 + 0.05]),
                'orientation': np.array([0, 0, 0, 1]),
                'type': 'top_grasp'
            }
            candidates.append(top_grasp)
            
            # Side face grasps
            face_offsets = [
                (width/2 + 0.05, 0, 0),      # Right face
                (-width/2 - 0.05, 0, 0),     # Left face
                (0, depth/2 + 0.05, 0),      # Front face
                (0, -depth/2 - 0.05, 0),     # Back face
            ]
            
            for offset in face_offsets:
                side_grasp = {
                    'position': np.array([
                        obj_pose[0] + offset[0],
                        obj_pose[1] + offset[1],
                        obj_pose[2] + offset[2]
                    ]),
                    'orientation': np.array([0, 0, 0, 1]),
                    'type': 'side_grasp'
                }
                candidates.append(side_grasp)
        
        return candidates
    
    def evaluate_grasp(self, grasp, object_info):
        """Evaluate the quality of a grasp"""
        # This is a simplified evaluation
        # In practice, you'd consider grasp stability, force closure, etc.
        
        # Factors affecting grasp quality:
        # 1. Distance to object center
        obj_center = object_info['pose']
        dist_to_center = np.linalg.norm(np.array(grasp['position']) - obj_center)
        
        # 2. Grasp type appropriateness
        grasp_type_score = 1.0 if grasp['type'] in ['top_grasp', 'side_grasp'] else 0.5
        
        # 3. Hand configuration feasibility
        config_feasibility = self.hand_model.check_configuration_feasibility(grasp)
        
        # Combine factors (simplified)
        score = 1.0 / (1.0 + dist_to_center) * grasp_type_score * config_feasibility
        
        return score
```

## In-Hand Manipulation

In-hand manipulation refers to adjusting the position and orientation of an object within the robot's grasp without releasing it. This is important for tasks requiring precise positioning.

```python
class InHandManipulator:
    def __init__(self, hand_model):
        self.hand_model = hand_model
    
    def reposition_object(self, initial_grasp, target_pose):
        """Reposition an object in the hand to a target pose"""
        # Calculate the transformation needed
        current_obj_pose = self.hand_model.get_object_pose_in_hand(initial_grasp)
        transformation = self.calculate_pose_transformation(current_obj_pose, target_pose)
        
        # Generate a sequence of finger movements to achieve the transformation
        manipulation_sequence = self.plan_manipulation_sequence(transformation)
        
        return manipulation_sequence
    
    def calculate_pose_transformation(self, current_pose, target_pose):
        """Calculate the transformation from current to target pose"""
        # Simplified transformation calculation
        # In practice, this would involve more complex kinematics
        pos_diff = target_pose[:3] - current_pose[:3]
        rot_diff = self.quaternion_difference(current_pose[3:], target_pose[3:])
        
        return {
            'position_change': pos_diff,
            'rotation_change': rot_diff
        }
    
    def quaternion_difference(self, q1, q2):
        """Calculate the difference between two quaternions"""
        # Convert to rotation matrices and find difference
        # Simplified implementation
        return np.array([0.0, 0.0, 0.0, 1.0])  # Placeholder
    
    def plan_manipulation_sequence(self, transformation):
        """Plan a sequence of finger movements for in-hand manipulation"""
        # Simplified sequence planning
        # In practice, this would involve detailed finger kinematics
        sequence = []
        
        # Example: Adjust thumb position
        sequence.append({
            'finger': 'thumb',
            'movement': transformation['position_change'] * 0.3,  # 30% of required movement
            'grip_force': 20.0  # Newtons
        })
        
        # Example: Adjust index finger
        sequence.append({
            'finger': 'index',
            'movement': transformation['position_change'] * 0.4,  # 40% of required movement
            'grip_force': 15.0  # Newtons
        })
        
        # Example: Adjust middle finger
        sequence.append({
            'finger': 'middle',
            'movement': transformation['position_change'] * 0.3,  # 30% of required movement
            'grip_force': 15.0  # Newtons
        })
        
        return sequence
```

## ROS 2 Manipulation Node

Let's implement a ROS 2 node that coordinates manipulation tasks:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image, PointCloud2
from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import String, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker
import numpy as np
import tf2_ros
import math

class ManipulationControllerNode(Node):
    def __init__(self):
        super().__init__('manipulation_controller_node')
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.object_pose_sub = self.create_subscription(
            Pose,
            '/detected_object_pose',
            self.object_pose_callback,
            10
        )
        
        self.manipulation_command_sub = self.create_subscription(
            String,
            '/manipulation_command',
            self.manipulation_command_callback,
            10
        )
        
        # Publishers
        self.left_arm_traj_pub = self.create_publisher(JointTrajectory, '/left_arm_controller/joint_trajectory', 10)
        self.right_arm_traj_pub = self.create_publisher(JointTrajectory, '/right_arm_controller/joint_trajectory', 10)
        self.gripper_cmd_pub = self.create_publisher(Float64MultiArray, '/gripper_commands', 10)
        self.visualization_pub = self.create_publisher(Marker, '/manipulation_visualization', 10)
        
        # TF buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Timer for control loop
        self.timer = self.create_timer(0.01, self.control_loop)
        
        # Initialize components
        self.grasp_planner = GraspPlanner(hand_model=None)  # Simplified
        self.in_hand_manipulator = InHandManipulator(hand_model=None)  # Simplified
        
        # Robot state
        self.joint_positions = {}
        self.object_pose = None
        self.current_task = None
        self.task_state = 'idle'  # idle, planning, executing, completed
        
    def joint_state_callback(self, msg):
        """Update joint positions from sensor data"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
    
    def object_pose_callback(self, msg):
        """Update detected object pose"""
        self.object_pose = np.array([
            msg.position.x, msg.position.y, msg.position.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ])
    
    def manipulation_command_callback(self, msg):
        """Handle manipulation commands"""
        command = msg.data
        self.get_logger().info(f'Received manipulation command: {command}')
        
        if command.startswith('grasp '):
            # Extract object information from command
            obj_info_str = command[6:]  # Remove 'grasp ' prefix
            obj_info = self.parse_object_info(obj_info_str)
            
            self.start_grasp_task(obj_info)
        elif command == 'release':
            self.start_release_task()
        elif command.startswith('move_object '):
            # Extract target pose
            pose_str = command[12:]  # Remove 'move_object ' prefix
            target_pose = self.parse_pose(pose_str)
            
            self.start_move_object_task(target_pose)
    
    def parse_object_info(self, info_str):
        """Parse object information from string"""
        # Simplified parsing - in practice, this would be more robust
        parts = info_str.split(',')
        return {
            'pose': np.array([float(parts[0]), float(parts[1]), float(parts[2])]),
            'shape': parts[3] if len(parts) > 3 else 'unknown',
            'size': np.array([float(parts[4]), float(parts[5]), float(parts[6])]) if len(parts) > 6 else np.array([0.1, 0.1, 0.1])
        }
    
    def parse_pose(self, pose_str):
        """Parse pose from string"""
        parts = pose_str.split(',')
        return np.array([float(x) for x in parts])
    
    def start_grasp_task(self, obj_info):
        """Start a grasping task"""
        self.get_logger().info('Planning grasp...')
        self.current_task = {
            'type': 'grasp',
            'object_info': obj_info
        }
        self.task_state = 'planning'
    
    def start_release_task(self):
        """Start a release task"""
        self.get_logger().info('Starting release task...')
        self.current_task = {
            'type': 'release'
        }
        self.task_state = 'executing'
        
        # Release the object by opening grippers
        self.execute_release()
    
    def start_move_object_task(self, target_pose):
        """Start a move object task"""
        self.get_logger().info('Starting move object task...')
        self.current_task = {
            'type': 'move_object',
            'target_pose': target_pose
        }
        self.task_state = 'planning'
    
    def control_loop(self):
        """Main control loop for manipulation tasks"""
        if self.task_state == 'planning':
            if self.current_task['type'] == 'grasp':
                self.plan_grasp()
            elif self.current_task['type'] == 'move_object':
                self.plan_object_move()
        elif self.task_state == 'executing':
            # In a real implementation, we would monitor execution
            # and update the state when the task is completed
            pass
    
    def plan_grasp(self):
        """Plan a grasp for the current object"""
        if self.current_task and self.current_task['type'] == 'grasp':
            obj_info = self.current_task['object_info']
            
            # Plan the grasp
            best_grasp = self.grasp_planner.plan_grasp(obj_info)
            
            if best_grasp:
                self.get_logger().info(f'Grasp planned: {best_grasp}')
                
                # Execute the grasp
                self.execute_grasp(best_grasp)
                self.task_state = 'executing'
            else:
                self.get_logger().error('Could not plan a suitable grasp')
                self.task_state = 'idle'
    
    def plan_object_move(self):
        """Plan how to move an object to a target pose"""
        if self.current_task and self.current_task['type'] == 'move_object':
            target_pose = self.current_task['target_pose']
            
            # For this example, we'll just move the end-effector to the target
            # In practice, this would involve path planning and coordination
            self.execute_move_to_pose(target_pose)
            self.task_state = 'executing'
    
    def execute_grasp(self, grasp):
        """Execute a planned grasp"""
        # Move arm to grasp position
        grasp_pose = grasp['position']
        grasp_orientation = grasp['orientation']
        
        # Calculate joint angles using inverse kinematics
        # This is simplified - in practice, you'd use a full IK solver
        joint_angles = self.calculate_arm_joints_for_pose(
            grasp_pose, grasp_orientation, arm='right'
        )
        
        # Move the arm to the grasp position
        self.move_arm_to_joints(joint_angles, arm='right')
        
        # Close the gripper
        self.close_gripper(arm='right')
        
        self.get_logger().info('Grasp executed')
    
    def execute_release(self):
        """Execute a release action"""
        # Open both grippers
        self.open_gripper(arm='right')
        self.open_gripper(arm='left')
        
        self.get_logger().info('Release executed')
        self.task_state = 'completed'
    
    def execute_move_to_pose(self, target_pose):
        """Move the end-effector to a target pose"""
        # Calculate joint angles for target pose
        joint_angles = self.calculate_arm_joints_for_pose(
            target_pose[:3], target_pose[3:], arm='right'
        )
        
        # Move the arm
        self.move_arm_to_joints(joint_angles, arm='right')
        
        self.get_logger().info('Move to pose executed')
    
    def calculate_arm_joints_for_pose(self, position, orientation, arm='right'):
        """Calculate joint angles for a given end-effector pose"""
        # This is a simplified implementation
        # In practice, you'd use inverse kinematics
        if arm == 'right':
            # Return a fixed set of joint angles for demonstration
            return [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]  # Example joint angles
        else:
            return [0.0, 0.5, -0.5, 0.0, -0.5, 0.0]  # Example joint angles for left arm
    
    def move_arm_to_joints(self, joint_angles, arm='right'):
        """Move an arm to specified joint angles"""
        # Create trajectory message
        trajectory = JointTrajectory()
        
        if arm == 'right':
            trajectory.joint_names = [
                'right_shoulder_pan_joint', 'right_shoulder_lift_joint',
                'right_elbow_joint', 'right_wrist_1_joint',
                'right_wrist_2_joint', 'right_wrist_3_joint'
            ]
        else:
            trajectory.joint_names = [
                'left_shoulder_pan_joint', 'left_shoulder_lift_joint',
                'left_elbow_joint', 'left_wrist_1_joint',
                'left_wrist_2_joint', 'left_wrist_3_joint'
            ]
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000  # 200ms
        
        trajectory.points = [point]
        
        # Publish trajectory
        if arm == 'right':
            self.right_arm_traj_pub.publish(trajectory)
        else:
            self.left_arm_traj_pub.publish(trajectory)
    
    def close_gripper(self, arm='right'):
        """Close the gripper on specified arm"""
        gripper_cmd = Float64MultiArray()
        if arm == 'right':
            gripper_cmd.data = [0.0, 0.0]  # Closed position for right gripper
        else:
            gripper_cmd.data = [0.0, 0.0]  # Closed position for left gripper
        self.gripper_cmd_pub.publish(gripper_cmd)
    
    def open_gripper(self, arm='right'):
        """Open the gripper on specified arm"""
        gripper_cmd = Float64MultiArray()
        if arm == 'right':
            gripper_cmd.data = [0.8, 0.8]  # Open position for right gripper
        else:
            gripper_cmd.data = [0.8, 0.8]  # Open position for left gripper
        self.gripper_cmd_pub.publish(gripper_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ManipulationControllerNode()
    
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

## Force Control and Compliance

For safe and effective manipulation, humanoid robots need to control the forces applied during interaction with objects:

```python
class ForceController:
    def __init__(self, stiffness=1000, damping=20, max_force=200):
        self.stiffness = stiffness  # N/m
        self.damping = damping      # Ns/m
        self.max_force = max_force  # N
    
    def calculate_impedance_force(self, desired_pos, current_pos, desired_vel, current_vel):
        """Calculate impedance control force"""
        pos_error = desired_pos - current_pos
        vel_error = desired_vel - current_vel
        
        force = self.stiffness * pos_error + self.damping * vel_error
        
        # Limit force magnitude
        force_magnitude = np.linalg.norm(force)
        if force_magnitude > self.max_force:
            force = force * (self.max_force / force_magnitude)
        
        return force
    
    def admittance_control(self, applied_force, current_pos, current_vel, dt):
        """Calculate position change based on applied force (admittance control)"""
        # Admittance: how much the system moves in response to force
        # This creates compliance in the system
        acceleration = applied_force / self.stiffness  # Simplified model
        new_vel = current_vel + acceleration * dt
        new_pos = current_pos + new_vel * dt
        
        return new_pos, new_vel
```

## Multi-Object Manipulation

Advanced manipulation might involve coordinating multiple objects or using both arms:

```python
class BimanualManipulator:
    def __init__(self):
        self.left_arm = ManipulatorArm(dh_params=[])  # Left arm model
        self.right_arm = ManipulatorArm(dh_params=[])  # Right arm model
        self.coordination_controller = None  # For coordinating both arms
    
    def coordinated_task(self, task_description):
        """Execute a task requiring both arms"""
        # Example: Opening a jar
        if task_description == 'open_jar':
            # Left arm holds the jar, right arm turns the lid
            left_arm_task = {
                'action': 'grasp',
                'object': 'jar_body',
                'pose': self.calculate_stabilizing_pose('jar')
            }
            
            right_arm_task = {
                'action': 'grasp_and_rotate',
                'object': 'jar_lid',
                'torque': 5.0  # Nm
            }
            
            # Execute coordinated movement
            self.execute_bimanual_task(left_arm_task, right_arm_task)
    
    def calculate_stabilizing_pose(self, object_name):
        """Calculate pose for stabilizing an object"""
        # Simplified implementation
        return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])  # Position + quaternion
    
    def execute_bimanual_task(self, left_task, right_task):
        """Execute a coordinated task with both arms"""
        # This would involve complex coordination to prevent conflicts
        # and achieve the desired outcome
        pass
```

## Conclusion

Manipulation and grasping are fundamental capabilities for humanoid robots, enabling them to interact with objects in their environment. By combining kinematic models, grasp planning, force control, and coordination strategies, we can create robots capable of complex manipulation tasks. In the next section, we'll explore multi-modal interaction, which combines manipulation with perception and natural language understanding.