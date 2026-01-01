---
title: Bipedal Locomotion & Balance Control
description: Understanding the principles of bipedal walking and balance control in humanoid robots
sidebar_position: 5
---

# Bipedal Locomotion & Balance Control

## Introduction to Bipedal Locomotion

Bipedal locomotion is one of the most challenging aspects of humanoid robotics. Unlike wheeled robots, humanoid robots must manage their center of mass (CoM) carefully to maintain balance while walking. This requires sophisticated control algorithms that can handle the dynamic nature of bipedal gait.

## Gait Generation

Humanoid walking patterns are typically modeled using various approaches:

1. **Predefined Trajectories**: Following predetermined joint angle patterns
2. **Central Pattern Generators (CPGs)**: Neural network-based oscillators
3. **Model-Based Approaches**: Using inverted pendulum models
4. **Learning-Based Approaches**: Reinforcement learning or imitation learning

## Inverted Pendulum Model

The Linear Inverted Pendulum Model (LIPM) is a common simplification for humanoid walking:

```python
import numpy as np
import math

class LinearInvertedPendulumModel:
    def __init__(self, com_height, gravity=9.81):
        self.com_height = com_height  # Center of mass height
        self.gravity = gravity
        self.omega = math.sqrt(gravity / com_height)
    
    def calculate_zmp(self, com_pos, com_vel, com_acc):
        """Calculate Zero Moment Point from CoM state"""
        zmp_x = com_pos[0] - (com_pos[2] - 0) / self.gravity * com_acc[0]
        zmp_y = com_pos[1] - (com_pos[2] - 0) / self.gravity * com_acc[1]
        return np.array([zmp_x, zmp_y, 0])
    
    def calculate_com_trajectory(self, zmp_trajectory, dt):
        """Calculate CoM trajectory from ZMP trajectory"""
        com_positions = []
        com_velocities = []
        com_accelerations = []
        
        # Initial conditions
        current_com = np.array([0.0, 0.0, self.com_height])
        current_vel = np.array([0.0, 0.0, 0.0])
        
        for zmp in zmp_trajectory:
            # Calculate CoM acceleration from ZMP
            com_acc_x = self.gravity / self.com_height * (current_com[0] - zmp[0])
            com_acc_y = self.gravity / self.com_height * (current_com[1] - zmp[1])
            com_acc = np.array([com_acc_x, com_acc_y, 0])
            
            # Update velocity and position
            current_vel += com_acc * dt
            current_com += current_vel * dt
            
            com_accelerations.append(com_acc)
            com_velocities.append(current_vel)
            com_positions.append(current_com)
        
        return com_positions, com_velocities, com_accelerations
```

## Walking Pattern Generation

A common approach to walking pattern generation is to use predefined trajectories for the center of mass, feet, and other body parts:

```python
class WalkingPatternGenerator:
    def __init__(self, step_length=0.3, step_height=0.05, step_time=1.0, com_height=0.8):
        self.step_length = step_length
        self.step_height = step_height
        self.step_time = step_time
        self.com_height = com_height
        self.lipm = LinearInvertedPendulumModel(com_height)
    
    def generate_foot_trajectory(self, start_pos, steps, support_leg='left'):
        """Generate foot trajectories for a given number of steps"""
        trajectories = {'left': [], 'right': []}
        
        # Initialize feet positions
        if support_leg == 'left':
            left_foot = np.array([0.0, 0.1, 0.0])  # Left foot slightly offset in Y
            right_foot = np.array([0.0, -0.1, 0.0])  # Right foot slightly offset in Y
        else:
            left_foot = np.array([0.0, 0.1, 0.0])
            right_foot = np.array([0.0, -0.1, 0.0])
        
        current_support_leg = support_leg
        time_step = 0.01  # 10ms time steps
        
        for step_idx in range(steps):
            # Calculate number of time steps for this step
            num_steps = int(self.step_time / time_step)
            
            for i in range(num_steps):
                t = i * time_step
                
                # Move the swing foot
                if current_support_leg == 'left':
                    # Right foot is swing foot
                    swing_foot = right_foot
                    support_foot = left_foot
                else:
                    # Left foot is swing foot
                    swing_foot = left_foot
                    support_foot = right_foot
                
                # Calculate swing foot position (cycloid trajectory)
                phase = t / self.step_time
                x = support_foot[0] + self.step_length * phase
                y = support_foot[1]  # Keep Y constant for now
                z = self.step_height * math.sin(math.pi * phase)  # Sinusoidal lift
                
                swing_foot[0] = x
                swing_foot[2] = z
                
                # Add to trajectories
                if current_support_leg == 'left':
                    trajectories['left'].append(support_foot.copy())
                    trajectories['right'].append(swing_foot.copy())
                else:
                    trajectories['left'].append(swing_foot.copy())
                    trajectories['right'].append(support_foot.copy())
            
            # Switch support leg for next step
            current_support_leg = 'right' if current_support_leg == 'left' else 'left'
            
            # Update starting positions for next step
            if step_idx % 2 == 0:
                # Update for odd steps
                left_foot[0] += self.step_length
                right_foot[0] += self.step_length
            else:
                # Update for even steps
                left_foot[0] += self.step_length
                right_foot[0] += self.step_length
        
        return trajectories
```

## Balance Control

Balance control is critical for stable walking. The Zero Moment Point (ZMP) is a key concept in humanoid balance:

```python
class BalanceController:
    def __init__(self, robot_mass, com_height, control_gain=10.0):
        self.robot_mass = robot_mass
        self.com_height = com_height
        self.control_gain = control_gain
        self.gravity = 9.81
        self.lipm = LinearInvertedPendulumModel(com_height)
        
        # PID controllers for X and Y directions
        self.pid_x = PIDController(kp=15.0, ki=0.1, kd=0.5)
        self.pid_y = PIDController(kp=15.0, ki=0.1, kd=0.5)
    
    def calculate_balance_correction(self, current_zmp, desired_zmp, dt):
        """Calculate corrective forces to maintain balance"""
        # Calculate error
        error_x = desired_zmp[0] - current_zmp[0]
        error_y = desired_zmp[1] - current_zmp[1]
        
        # Apply PID control
        correction_x = self.pid_x.update(error_x, dt)
        correction_y = self.pid_y.update(error_y, dt)
        
        return np.array([correction_x, correction_y, 0.0])
    
    def adjust_com_for_balance(self, current_com, current_zmp, desired_zmp, dt):
        """Adjust CoM position to maintain balance"""
        # Calculate required CoM adjustment
        correction = self.calculate_balance_correction(current_zmp, desired_zmp, dt)
        
        # Apply correction to CoM
        target_com = current_com.copy()
        target_com[0] += correction[0] * dt * 0.1  # Scale factor
        target_com[1] += correction[1] * dt * 0.1  # Scale factor
        
        return target_com

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
    
    def update(self, error, dt):
        if dt <= 0.0:
            return 0.0
        
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        
        return output
```

## ROS 2 Walking Controller

Let's implement a ROS 2 node that coordinates walking and balance:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Float64MultiArray, Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

class WalkingControllerNode(Node):
    def __init__(self):
        super().__init__('walking_controller_node')
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.com_sub = self.create_subscription(
            Point,
            '/center_of_mass',
            self.com_callback,
            10
        )
        
        self.zmp_sub = self.create_subscription(
            Point,
            '/zero_moment_point',
            self.zmp_callback,
            10
        )
        
        self.walk_command_sub = self.create_subscription(
            Bool,
            '/walk_command',
            self.walk_command_callback,
            10
        )
        
        # Publishers
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)
        self.com_ref_pub = self.create_publisher(Point, '/com_reference', 10)
        self.zmp_ref_pub = self.create_publisher(Point, '/zmp_reference', 10)
        
        # Timer for walking control
        self.timer = self.create_timer(0.01, self.walk_control_loop)
        
        # Initialize walking components
        self.pattern_generator = WalkingPatternGenerator()
        self.balance_controller = BalanceController(robot_mass=50.0, com_height=0.8)
        
        # Walking state
        self.joint_positions = {}
        self.current_com = np.array([0.0, 0.0, 0.8])
        self.current_zmp = np.array([0.0, 0.0, 0.0])
        self.is_walking = False
        self.step_count = 0
        
    def joint_state_callback(self, msg):
        """Update joint positions from sensor data"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
    
    def com_callback(self, msg):
        """Update center of mass position"""
        self.current_com = np.array([msg.x, msg.y, msg.z])
    
    def zmp_callback(self, msg):
        """Update zero moment point"""
        self.current_zmp = np.array([msg.x, msg.y, msg.z])
    
    def walk_command_callback(self, msg):
        """Handle walk start/stop commands"""
        self.is_walking = msg.data
        if self.is_walking:
            self.get_logger().info('Starting to walk...')
            self.step_count = 0
        else:
            self.get_logger().info('Stopping walk...')
    
    def walk_control_loop(self):
        """Main walking control loop"""
        if not self.is_walking:
            return
        
        # Generate walking pattern for current step
        # In a real implementation, this would be pre-computed
        dt = 0.01  # 100Hz control loop
        
        # Generate desired ZMP trajectory (simplified)
        # For now, just a simple pattern
        desired_zmp = np.array([0.01 * self.step_count, 0.0, 0.0])
        
        # Calculate balance correction
        balance_correction = self.balance_controller.calculate_balance_correction(
            self.current_zmp, desired_zmp, dt
        )
        
        # Generate joint trajectories based on walking pattern
        # This is a simplified version - in reality, you'd use inverse kinematics
        # to calculate joint angles that achieve the desired foot positions
        joint_trajectory = self.generate_joint_trajectory()
        
        # Publish the trajectory
        self.trajectory_pub.publish(joint_trajectory)
        
        # Publish reference points
        com_ref_msg = Point()
        com_ref_msg.x = desired_zmp[0]  # Simplified
        com_ref_msg.y = desired_zmp[1]
        com_ref_msg.z = self.balance_controller.com_height
        self.com_ref_pub.publish(com_ref_msg)
        
        zmp_ref_msg = Point()
        zmp_ref_msg.x = desired_zmp[0]
        zmp_ref_msg.y = desired_zmp[1]
        zmp_ref_msg.z = 0.0
        self.zmp_ref_pub.publish(zmp_ref_msg)
        
        # Increment step counter periodically
        if self.get_clock().now().nanoseconds % 1000000000 < 10000000:  # Every ~1 second
            self.step_count += 1
    
    def generate_joint_trajectory(self):
        """Generate a joint trajectory message for walking"""
        # This is a simplified implementation
        # In practice, you'd use inverse kinematics to calculate joint angles
        # that achieve the desired foot positions from the walking pattern
        
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        
        # Simplified joint angles for walking motion
        # In reality, these would come from inverse kinematics
        point.positions = [
            0.1,   # left_hip_joint
            -0.2,  # left_knee_joint
            0.1,   # left_ankle_joint
            -0.1,  # right_hip_joint
            0.2,   # right_knee_joint
            -0.1   # right_ankle_joint
        ]
        
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 10000000  # 10ms
        
        trajectory.points = [point]
        
        return trajectory

def main(args=None):
    rclpy.init(args=args)
    node = WalkingControllerNode()
    
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

## Advanced Balance Strategies

For more robust balance, humanoid robots employ several advanced strategies:

1. **Capture Point**: A point where the robot can come to a stop
2. **Push Recovery**: Strategies to recover from external disturbances
3. **Ankle, Hip, and Step Strategies**: Different mechanisms for balance recovery

```python
class AdvancedBalanceController:
    def __init__(self, com_height, max_ankle_torque=100.0):
        self.com_height = com_height
        self.max_ankle_torque = max_ankle_torque
        self.gravity = 9.81
        self.omega = math.sqrt(self.gravity / self.com_height)
        
    def calculate_capture_point(self, com_pos, com_vel):
        """Calculate the capture point where robot can come to a stop"""
        capture_point_x = com_pos[0] + com_vel[0] / self.omega
        capture_point_y = com_pos[1] + com_vel[1] / self.omega
        return np.array([capture_point_x, capture_point_y, 0.0])
    
    def ankle_strategy(self, com_error, com_vel_error):
        """Ankle strategy for small perturbations"""
        # Apply ankle torques to correct CoM position
        # Torque is proportional to CoM error and velocity error
        ankle_torque_x = 50.0 * com_error[0] + 10.0 * com_vel_error[0]
        ankle_torque_y = 50.0 * com_error[1] + 10.0 * com_vel_error[1]
        
        # Limit torque to maximum ankle capability
        ankle_torque_x = max(min(ankle_torque_x, self.max_ankle_torque), -self.max_ankle_torque)
        ankle_torque_y = max(min(ankle_torque_y, self.max_ankle_torque), -self.max_ankle_torque)
        
        return np.array([ankle_torque_x, ankle_torque_y, 0.0])
    
    def hip_strategy(self, com_error, com_vel_error):
        """Hip strategy for larger perturbations"""
        # Lean the torso to shift CoM
        hip_torque_x = 80.0 * com_error[0] + 15.0 * com_vel_error[0]
        hip_torque_y = 80.0 * com_error[1] + 15.0 * com_vel_error[1]
        
        return np.array([hip_torque_x, hip_torque_y, 0.0])
    
    def step_strategy(self, capture_point, foot_positions):
        """Determine if a step is needed and where to step"""
        # Check if capture point is outside support polygon
        # For simplicity, assume rectangular support area
        support_margin = 0.05  # 5cm margin
        
        # Check if capture point is outside support region
        left_foot_x, left_foot_y = foot_positions['left']
        right_foot_x, right_foot_y = foot_positions['right']
        
        # Calculate support polygon bounds
        min_x = min(left_foot_x, right_foot_x) - support_margin
        max_x = max(left_foot_x, right_foot_x) + support_margin
        min_y = min(left_foot_y, right_foot_y) - support_margin
        max_y = max(left_foot_y, right_foot_y) + support_margin
        
        # If capture point is outside support polygon, stepping is needed
        needs_step = (capture_point[0] < min_x or capture_point[0] > max_x or
                      capture_point[1] < min_y or capture_point[1] > max_y)
        
        if needs_step:
            # Calculate where to step (simplified - step toward capture point)
            step_x = capture_point[0]
            step_y = capture_point[1]
            return True, np.array([step_x, step_y, 0.0])
        else:
            return False, None
```

## Conclusion

Bipedal locomotion and balance control are complex but essential aspects of humanoid robotics. By combining dynamic models like the Linear Inverted Pendulum with advanced control strategies, we can create stable walking robots. The next section will explore manipulation and grasping, which requires integrating the locomotion system with arm control for coordinated tasks.