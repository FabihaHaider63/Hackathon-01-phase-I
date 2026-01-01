---
title: Humanoid Kinematics & Dynamics
description: Understanding the mathematical models governing humanoid robot movement and force interactions
sidebar_position: 4
---

# Humanoid Kinematics & Dynamics

## Introduction to Humanoid Kinematics

Humanoid kinematics is the study of motion in humanoid robots without considering the forces that cause the motion. It involves understanding the geometric relationships between different body parts and how they move relative to each other. For humanoid robots, this is particularly complex due to the large number of degrees of freedom (DOF) and the need for stable locomotion.

## Forward Kinematics

Forward kinematics calculates the position and orientation of the end-effector (e.g., hand or foot) given the joint angles. For humanoid robots, we typically use the Denavit-Hartenberg (DH) convention or product of exponentials (POE) to model the kinematic chains.

Let's consider a simplified humanoid arm with shoulder, elbow, and wrist joints:

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

def forward_kinematics_arm(joint_angles, link_lengths):
    """Calculate the end-effector position for a simplified arm"""
    # Joint angles [shoulder_yaw, shoulder_pitch, elbow_pitch, wrist_pitch]
    # Link lengths [upper_arm_length, forearm_length, hand_length]
    
    # Base transformation
    T = np.eye(4)
    
    # Shoulder yaw joint
    T = T @ dh_transform(0, -math.pi/2, 0, joint_angles[0])
    
    # Shoulder pitch joint
    T = T @ dh_transform(0, math.pi/2, 0, joint_angles[1])
    
    # Upper arm
    T = T @ dh_transform(link_lengths[0], 0, 0, 0)
    
    # Elbow pitch joint
    T = T @ dh_transform(0, -math.pi/2, 0, joint_angles[2])
    
    # Forearm
    T = T @ dh_transform(link_lengths[1], 0, 0, 0)
    
    # Wrist pitch joint
    T = T @ dh_transform(0, 0, 0, joint_angles[3])
    
    # Hand
    T = T @ dh_transform(link_lengths[2], 0, 0, 0)
    
    return T

# Example usage
joint_angles = [0.1, 0.5, -0.3, 0.2]  # radians
link_lengths = [0.3, 0.25, 0.1]  # meters
end_effector_pose = forward_kinematics_arm(joint_angles, link_lengths)
print("End-effector pose:\n", end_effector_pose)
```

## Inverse Kinematics

Inverse kinematics (IK) solves the opposite problem: given a desired end-effector position and orientation, find the joint angles that achieve it. This is crucial for humanoid robots to reach for objects or maintain balance.

```python
def jacobian_transpose_ik(joint_angles, target_pos, link_lengths, max_iterations=100, tolerance=1e-4):
    """Solve inverse kinematics using Jacobian transpose method"""
    
    current_angles = joint_angles.copy()
    
    for i in range(max_iterations):
        # Calculate current end-effector position
        current_pose = forward_kinematics_arm(current_angles, link_lengths)
        current_pos = current_pose[:3, 3]
        
        # Calculate error
        error = target_pos - current_pos
        
        if np.linalg.norm(error) < tolerance:
            break
        
        # Calculate Jacobian (simplified for position only)
        J = calculate_jacobian(current_angles, link_lengths)
        
        # Update joint angles using Jacobian transpose
        dtheta = np.dot(J.T, error)
        current_angles += 0.1 * dtheta  # Learning rate
    
    return current_angles

def calculate_jacobian(joint_angles, link_lengths):
    """Calculate the Jacobian matrix for the arm"""
    # Simplified Jacobian calculation
    # In practice, you'd use more sophisticated methods
    
    # Extract joint angles
    q1, q2, q3, q4 = joint_angles
    l1, l2, l3 = link_lengths
    
    # Calculate positions of each joint in world frame
    # Shoulder position
    x1 = 0
    y1 = 0
    
    # Elbow position
    x2 = l1 * math.cos(q1 + q2)
    y2 = l1 * math.sin(q1 + q2)
    
    # Wrist position
    x3 = x2 + l2 * math.cos(q1 + q2 + q3)
    y3 = y2 + l2 * math.sin(q1 + q2 + q3)
    
    # End-effector position
    x4 = x3 + l3 * math.cos(q1 + q2 + q3 + q4)
    y4 = y3 + l3 * math.sin(q1 + q2 + q3 + q4)
    
    # Jacobian matrix (partial derivatives of end-effector position w.r.t. joint angles)
    J = np.array([
        [-l1*math.sin(q1+q2) - l2*math.sin(q1+q2+q3) - l3*math.sin(q1+q2+q3+q4),  # dx/dq1
         -l1*math.sin(q1+q2) - l2*math.sin(q1+q2+q3) - l3*math.sin(q1+q2+q3+q4),  # dx/dq2
         -l2*math.sin(q1+q2+q3) - l3*math.sin(q1+q2+q3+q4),  # dx/dq3
         -l3*math.sin(q1+q2+q3+q4)],  # dx/dq4
         
         [l1*math.cos(q1+q2) + l2*math.cos(q1+q2+q3) + l3*math.cos(q1+q2+q3+q4),   # dy/dq1
          l1*math.cos(q1+q2) + l2*math.cos(q1+q2+q3) + l3*math.cos(q1+q2+q3+q4),   # dy/dq2
          l2*math.cos(q1+q2+q3) + l3*math.cos(q1+q2+q3+q4),   # dy/dq3
          l3*math.cos(q1+q2+q3+q4)]   # dy/dq4
    ])
    
    return J
```

## Center of Mass and Balance

For humanoid robots, maintaining balance is critical. The center of mass (CoM) position needs to be controlled to keep the robot stable:

```python
class BalanceController:
    def __init__(self, robot_masses, robot_positions):
        """
        Initialize balance controller
        :param robot_masses: List of masses for each body part
        :param robot_positions: List of positions for each body part [x, y, z]
        """
        self.masses = robot_masses
        self.positions = robot_positions
    
    def calculate_com(self):
        """Calculate the center of mass of the robot"""
        total_mass = sum(self.masses)
        
        com_x = sum(mass * pos[0] for mass, pos in zip(self.masses, self.positions)) / total_mass
        com_y = sum(mass * pos[1] for mass, pos in zip(self.masses, self.positions)) / total_mass
        com_z = sum(mass * pos[2] for mass, pos in zip(self.masses, self.positions)) / total_mass
        
        return np.array([com_x, com_y, com_z])
    
    def calculate_zmp(self, com_pos, com_vel, com_acc, gravity=9.81):
        """
        Calculate Zero Moment Point (ZMP)
        This is a simplified 2D version
        """
        # ZMP_x = com_x - (com_z - zmp_z) / g * com_acc_x
        # ZMP_y = com_y - (com_z - zmp_z) / g * com_acc_y
        
        zmp_z = 0  # Assume ZMP is on ground plane
        
        zmp_x = com_pos[0] - (com_pos[2] - zmp_z) / gravity * com_acc[0]
        zmp_y = com_pos[1] - (com_pos[2] - zmp_z) / gravity * com_acc[1]
        
        return np.array([zmp_x, zmp_y, zmp_z])
    
    def is_stable(self, zmp, support_polygon):
        """
        Check if the ZMP is within the support polygon
        :param zmp: Zero Moment Point [x, y, z]
        :param support_polygon: List of vertices of the support polygon
        :return: True if stable, False otherwise
        """
        # Simplified point-in-polygon test for 2D
        x, y = zmp[0], zmp[1]
        n = len(support_polygon)
        inside = False
        
        p1x, p1y = support_polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = support_polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        
        return inside
```

## Dynamics Modeling

Dynamics modeling considers the forces and torques required to generate the desired motion. The equation of motion for a humanoid robot is given by:

```
M(q)q'' + C(q, q')q' + g(q) = τ
```

Where:
- M(q) is the mass matrix
- C(q, q') represents Coriolis and centrifugal forces
- g(q) represents gravitational forces
- τ is the vector of joint torques
- q, q', q'' are joint positions, velocities, and accelerations

```python
class HumanoidDynamics:
    def __init__(self, robot_urdf):
        """Initialize dynamics model from URDF"""
        self.urdf = robot_urdf
        # In practice, you'd parse the URDF and build the dynamics model
        # For this example, we'll use simplified parameters
    
    def compute_mass_matrix(self, joint_positions):
        """Compute the mass matrix M(q)"""
        # This is a simplified example
        # In practice, you'd use recursive Newton-Euler or other methods
        n = len(joint_positions)  # Number of joints
        M = np.zeros((n, n))
        
        # Fill in mass matrix elements based on robot configuration
        # This is highly simplified - real implementation would be much more complex
        for i in range(n):
            M[i, i] = 1.0  # Simplified diagonal elements
        
        return M
    
    def compute_coriolis_gravity(self, joint_positions, joint_velocities):
        """Compute C(q, q')q' + g(q)"""
        # Simplified computation
        n = len(joint_positions)
        Cg = np.zeros(n)
        
        # Add simplified Coriolis and gravity terms
        for i in range(n):
            Cg[i] = 0.1 * joint_velocities[i]  # Simplified Coriolis term
            Cg[i] += 0.5 * math.sin(joint_positions[i])  # Simplified gravity term
        
        return Cg
    
    def inverse_dynamics(self, joint_positions, joint_velocities, joint_accelerations):
        """Compute required joint torques for desired motion"""
        M = self.compute_mass_matrix(joint_positions)
        Cg = self.compute_coriolis_gravity(joint_positions, joint_velocities)
        
        # τ = M(q)q'' + C(q, q')q' + g(q)
        torques = M @ joint_accelerations + Cg
        
        return torques
    
    def forward_dynamics(self, joint_positions, joint_velocities, joint_torques):
        """Compute accelerations from applied torques"""
        M = self.compute_mass_matrix(joint_positions)
        Cg = self.compute_coriolis_gravity(joint_positions, joint_velocities)
        
        # q'' = M^(-1)(τ - C(q, q')q' - g(q))
        joint_accelerations = np.linalg.inv(M) @ (joint_torques - Cg)
        
        return joint_accelerations
```

## ROS 2 Implementation

Now let's implement these concepts in a ROS 2 node:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Float64MultiArray
import numpy as np

class KinematicsDynamicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_dynamics_node')
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publishers
        self.com_publisher = self.create_publisher(Point, '/center_of_mass', 10)
        self.zmp_publisher = self.create_publisher(Point, '/zero_moment_point', 10)
        self.joint_torques_publisher = self.create_publisher(Float64MultiArray, '/joint_torques', 10)
        
        # Timer for periodic calculations
        self.timer = self.create_timer(0.01, self.update_kinematics_dynamics)
        
        # Robot state
        self.joint_positions = {}
        self.joint_velocities = {}
        self.joint_effort = {}
        
        # Robot model parameters (simplified)
        self.link_lengths = [0.3, 0.25, 0.1]  # For our example arm
        self.robot_masses = [2.0, 1.5, 1.0, 10.0]  # Various body parts
        self.robot_positions = [
            [0.0, 0.0, 0.5],   # Head
            [0.0, 0.0, 0.3],   # Torso
            [0.1, 0.0, 0.4],   # Arm
            [0.0, 0.0, 0.1]    # Base
        ]
        
        self.balance_controller = BalanceController(self.robot_masses, self.robot_positions)
        
    def joint_state_callback(self, msg):
        """Update joint state from sensor data"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]
            if i < len(msg.effort):
                self.joint_effort[name] = msg.effort[i]
    
    def update_kinematics_dynamics(self):
        """Periodically update kinematics and dynamics calculations"""
        # Calculate center of mass
        com = self.balance_controller.calculate_com()
        
        # Publish CoM
        com_msg = Point()
        com_msg.x = com[0]
        com_msg.y = com[1]
        com_msg.z = com[2]
        self.com_publisher.publish(com_msg)
        
        # Calculate ZMP (simplified with assumed values)
        # In a real implementation, you'd calculate velocity and acceleration from joint states
        zmp = self.balance_controller.calculate_zmp(com, np.zeros(3), np.zeros(3))
        
        # Publish ZMP
        zmp_msg = Point()
        zmp_msg.x = zmp[0]
        zmp_msg.y = zmp[1]
        zmp_msg.z = zmp[2]
        self.zmp_publisher.publish(zmp_msg)
        
        # Check stability
        # Define a simple support polygon (for a biped in double support)
        support_polygon = [
            [-0.1, -0.05],  # Left foot back left
            [-0.1, 0.05],   # Left foot back right
            [0.1, 0.05],    # Left foot front right
            [0.1, -0.05]    # Left foot front left
        ]
        
        is_stable = self.balance_controller.is_stable(zmp, support_polygon)
        if not is_stable:
            self.get_logger().warn('Robot is unstable!')
        
        # Compute required joint torques (simplified)
        # In practice, you'd use desired accelerations to compute torques
        joint_names = list(self.joint_positions.keys())
        current_positions = [self.joint_positions[name] for name in joint_names]
        current_velocities = [self.joint_velocities.get(name, 0.0) for name in joint_names]
        desired_accelerations = [0.0] * len(current_positions)  # For now, zero acceleration
        
        dynamics = HumanoidDynamics(None)  # Simplified
        torques = dynamics.inverse_dynamics(current_positions, current_velocities, desired_accelerations)
        
        # Publish joint torques
        torques_msg = Float64MultiArray()
        torques_msg.data = torques.tolist()
        self.joint_torques_publisher.publish(torques_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsDynamicsNode()
    
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

## Control Strategies

For humanoid robots, several control strategies are used to manage the complex kinematics and dynamics:

1. **Computed Torque Control**: Uses dynamic model to compute required joint torques
2. **Impedance Control**: Controls the robot's mechanical impedance
3. **Operational Space Control**: Controls end-effector position and forces directly
4. **Whole-Body Control**: Optimizes multiple tasks simultaneously

## Conclusion

Understanding humanoid kinematics and dynamics is fundamental to controlling these complex robots. The mathematical models allow us to predict and control motion, maintain balance, and execute complex tasks. In the next section, we'll explore how these principles apply specifically to bipedal locomotion and balance control.