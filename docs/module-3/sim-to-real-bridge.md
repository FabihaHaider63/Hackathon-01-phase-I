---
title: Sim-to-Real Bridge
description: Transferring trained models and behaviors from simulation to real humanoid robots
sidebar_position: 7
---

# Sim-to-Real Bridge

## Overview

The sim-to-real transfer is the critical process of taking behaviors, control policies, and perception models trained in simulation and successfully deploying them on physical humanoid robots. This process is challenging due to the "reality gap" between simulated and real environments, but NVIDIA Isaac provides several tools and techniques to make this transition as smooth as possible.

## Understanding the Reality Gap

### Sources of the Reality Gap

The sim-to-real gap occurs due to differences between simulation and reality:

- **Visual Differences**: Lighting, textures, and rendering discrepancies
- **Physical Differences**: Dynamics, friction, and physical properties
- **Sensor Differences**: Noise, latency, and accuracy variations
- **Actuator Differences**: Motor dynamics and control precision

### Quantifying the Gap

Understanding the specific differences helps in selecting appropriate transfer techniques:

```python
# Example: Quantifying reality gap in humanoid locomotion
import numpy as np
import matplotlib.pyplot as plt

class RealityGapAnalyzer:
    def __init__(self):
        self.sim_data = {}
        self.real_data = {}
        
    def collect_data(self, sim_robot, real_robot, task):
        # Collect data from both simulation and real robot
        self.sim_data = self.run_task(sim_robot, task)
        self.real_data = self.run_task(real_robot, task)
        
    def analyze_gap(self):
        # Compare key metrics between sim and real
        metrics = {
            'joint_angles_diff': np.mean(np.abs(
                self.sim_data['joint_angles'] - self.real_data['joint_angles']
            )),
            'velocity_diff': np.mean(np.abs(
                self.sim_data['velocities'] - self.real_data['velocities']
            )),
            'balance_diff': np.abs(
                self.sim_data['balance_score'] - self.real_data['balance_score']
            ),
            'energy_efficiency_diff': np.abs(
                self.sim_data['energy'] - self.real_data['energy']
            )
        }
        return metrics
        
    def visualize_gap(self):
        # Create visualizations comparing sim vs real
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
        # Joint angle comparison
        axes[0,0].plot(self.sim_data['joint_angles'], label='Simulation')
        axes[0,0].plot(self.real_data['joint_angles'], label='Real')
        axes[0,0].set_title('Joint Angle Comparison')
        axes[0,0].legend()
        
        # Velocity comparison
        axes[0,1].plot(self.sim_data['velocities'], label='Simulation')
        axes[0,1].plot(self.real_data['velocities'], label='Real')
        axes[0,1].set_title('Velocity Comparison')
        axes[0,1].legend()
        
        # Balance score comparison
        axes[1,0].plot(self.sim_data['balance_score'], label='Simulation')
        axes[1,0].plot(self.real_data['balance_score'], label='Real')
        axes[1,0].set_title('Balance Score Comparison')
        axes[1,0].legend()
        
        # Energy consumption comparison
        axes[1,1].bar(['Simulation', 'Real'], 
                     [self.sim_data['avg_energy'], self.real_data['avg_energy']])
        axes[1,1].set_title('Energy Consumption Comparison')
        
        plt.tight_layout()
        plt.show()
```

## Domain Randomization

### Visual Domain Randomization

Making models robust to visual differences:

```python
# Implementing visual domain randomization in Isaac Sim
import omni
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.materials import PreviewSurface
import numpy as np
from pxr import Gf

class VisualDomainRandomizer:
    def __init__(self, env):
        self.env = env
        self.domain_params = {
            'lighting_range': [0.5, 2.0],  # Intensity range
            'color_range': [0.0, 1.0],     # Color range
            'texture_scale_range': [0.8, 1.2],  # Texture scaling
            'camera_noise_range': [0.0, 0.05]   # Noise level range
        }
        
    def randomize_visual_properties(self):
        # Randomize lighting conditions
        self._randomize_lighting()
        
        # Randomize material properties
        self._randomize_materials()
        
        # Randomize camera properties
        self._randomize_camera()
        
    def _randomize_lighting(self):
        # Get all lights in the scene
        light_prims = self.env.scene._lights
        
        for light_prim in light_prims:
            # Randomize light intensity
            random_intensity = np.random.uniform(
                self.domain_params['lighting_range'][0],
                self.domain_params['lighting_range'][1]
            )
            light_prim.intensity = random_intensity
            
            # Randomize light color
            random_color = Gf.Vec3f(
                np.random.uniform(0.0, 1.0),
                np.random.uniform(0.0, 1.0),
                np.random.uniform(0.0, 1.0)
            )
            light_prim.color = random_color
    
    def _randomize_materials(self):
        # Get all objects in the scene
        objects = self.env.scene._objects
        
        for obj_name, obj in objects.items():
            # Randomize material properties
            material_path = obj.prim_path + "/Looks/omni_universal_material"
            material_prim = get_prim_at_path(material_path)
            
            # Randomize color
            random_color = np.random.uniform(
                self.domain_params['color_range'][0],
                self.domain_params['color_range'][1],
                size=3
            )
            
            # Apply new material properties
            preview_surface = PreviewSurface(
                prim_path=material_path,
                color=random_color
            )
    
    def _randomize_camera(self):
        # Add noise to camera images
        from omni.isaac.sensor import Camera
        
        # For each camera, add random noise parameters
        # This would be done during camera initialization
        pass
```

### Physical Domain Randomization

Making models robust to physical differences:

```python
# Physical domain randomization implementation
class PhysicalDomainRandomizer:
    def __init__(self, env):
        self.env = env
        self.params = {
            'mass_range': [0.8, 1.2],        # ±20% mass variation
            'friction_range': [0.1, 3.0],    # Wide friction range
            'damping_range': [0.5, 1.5],     # Damping variation
            'com_offset_range': [-0.05, 0.05] # Center of mass offset
        }
    
    def randomize_physical_properties(self):
        # Randomize mass properties
        self._randomize_mass()
        
        # Randomize friction
        self._randomize_friction()
        
        # Randomize joint damping
        self._randomize_damping()
        
        # Randomize center of mass
        self._randomize_com()
    
    def _randomize_mass(self):
        # Randomize link masses for all robots
        for robot in self.env.robots:
            for link in robot.links:
                original_mass = link.mass
                random_factor = np.random.uniform(
                    self.params['mass_range'][0],
                    self.params['mass_range'][1]
                )
                link.mass = original_mass * random_factor
    
    def _randomize_friction(self):
        # Randomize friction coefficients
        for robot in self.env.robots:
            for joint in robot.joints:
                random_friction = np.random.uniform(
                    self.params['friction_range'][0],
                    self.params['friction_range'][1]
                )
                joint.friction = random_friction
    
    def _randomize_damping(self):
        # Randomize joint damping
        for robot in self.env.robots:
            for joint in robot.joints:
                random_damping = joint.damping * np.random.uniform(
                    self.params['damping_range'][0],
                    self.params['damping_range'][1]
                )
                joint.damping = random_damping
    
    def _randomize_com(self):
        # Randomize center of mass offset
        for robot in self.env.robots:
            for link in robot.links:
                random_offset = np.random.uniform(
                    self.params['com_offset_range'][0],
                    self.params['com_offset_range'][1],
                    size=3
                )
                link.center_of_mass_offset = random_offset
```

## System Identification

### Calibrating Simulation to Reality

System identification helps match simulation parameters to real robot behavior:

```python
# System identification for humanoid robot
import scipy.optimize as opt
import control  # python-control package

class SystemIdentifier:
    def __init__(self, real_robot, sim_robot):
        self.real_robot = real_robot
        self.sim_robot = sim_robot
        self.param_bounds = [
            (0.5, 2.0),    # Mass multipliers
            (0.1, 5.0),    # Friction range
            (0.1, 2.0),    # Damping range
            (-0.1, 0.1)    # COM offset range
        ]
    
    def identify_parameters(self, input_signal, real_output):
        """
        Identify simulation parameters to match real robot behavior
        """
        # Objective function to minimize difference between sim and real
        def objective_function(params):
            # Update simulation with new parameters
            self.update_sim_params(params)
            
            # Run simulation with same input as real robot
            sim_output = self.sim_robot.run_trajectory(input_signal)
            
            # Calculate difference
            error = np.mean(np.square(real_output - sim_output))
            return error
        
        # Initial guess
        initial_params = [1.0, 1.0, 1.0, 0.0]
        
        # Optimize parameters
        result = opt.minimize(
            objective_function,
            initial_params,
            bounds=self.param_bounds,
            method='L-BFGS-B'
        )
        
        return result.x
    
    def update_sim_params(self, params):
        # Update simulation with identified parameters
        mass_mult, friction_mult, damping_mult, com_offset = params
        
        # Apply to simulation
        self.sim_robot.mass_multiplier = mass_mult
        self.sim_robot.friction_multiplier = friction_mult
        self.sim_robot.damping_multiplier = damping_mult
        self.sim_robot.com_offset = com_offset
```

## Control Transfer Techniques

### Robust Control Design

Implementing controllers that work well in both simulation and reality:

```python
# Robust control for sim-to-real transfer
import torch
import numpy as np

class RobustController:
    def __init__(self, nominal_model_params):
        self.nominal_params = nominal_model_params
        self.uncertainty_bounds = {}
        self.controller = self._design_controller()
        
    def _design_controller(self):
        # Design a robust controller that can handle uncertainties
        # For example, a model reference adaptive controller (MRAC)
        
        # Initialize controller parameters
        self.theta = np.random.normal(0, 0.01, size=(60,))  # Controller parameters
        
        # Adaptive gain
        self.gamma = 1.0
        
        return self._control_law
    
    def _control_law(self, state, reference):
        # Robust control law with adaptation
        # e = tracking_error
        e = reference - state[:len(reference)]
        
        # Update controller parameters using adaptation law
        phi = self._regression_vector(state, reference)
        self.theta += self.gamma * np.outer(e, phi).flatten()
        
        # Compute control action
        u = np.dot(phi, self.theta.reshape(phi.shape[1], -1))
        
        return u.flatten()
    
    def _regression_vector(self, state, reference):
        # Create regression vector for adaptive control
        # This would include state, reference, and their derivatives
        n = len(state)
        m = len(reference)
        
        # Create regression matrix
        phi = np.zeros((m, n + m))  # State + reference dimensions
        phi[:, :n] = np.eye(m, n) if n >= m else np.hstack([np.eye(n), np.zeros((n, m-n))])[:m, :]
        phi[:, n:] = np.eye(m) if m <= n else np.eye(n, m)[:m, :]
        
        return phi
```

### Model Predictive Control (MPC)

MPC can be tuned to be robust to model uncertainties:

```python
# MPC for robust sim-to-real transfer
import cvxpy as cp
import numpy as np

class RobustMPC:
    def __init__(self, n_states, n_controls, prediction_horizon=10):
        self.n_states = n_states
        self.n_controls = n_controls
        self.N = prediction_horizon
        
        # Uncertainty bounds
        self.A_uncertainty_bound = 0.1  # 10% uncertainty in A matrix
        self.B_uncertainty_bound = 0.1  # 10% uncertainty in B matrix
    
    def solve_mpc(self, x0, reference_trajectory, Q, R, A_nominal, B_nominal):
        # Define optimization variables
        X = cp.Variable((self.n_states, self.N+1))  # State trajectory
        U = cp.Variable((self.n_controls, self.N))  # Control trajectory
        
        # Objective function
        cost = 0
        for k in range(self.N):
            cost += cp.quad_form(X[:, k] - reference_trajectory[k], Q)
            cost += cp.quad_form(U[:, k], R)
        cost += cp.quad_form(X[:, self.N] - reference_trajectory[self.N], Q)
        
        # Constraints
        constraints = [X[:, 0] == x0]
        
        # Robust constraints considering uncertainty
        for k in range(self.N):
            # Nominal dynamics
            x_next = A_nominal @ X[:, k] + B_nominal @ U[:, k]
            
            # Add robustness margin for uncertainties
            A_uncertainty = self.A_uncertainty_bound * np.ones_like(A_nominal)
            B_uncertainty = self.B_uncertainty_bound * np.ones_like(B_nominal)
            
            # Robust constraint: x[k+1] should be in reachable set
            # Simplified: add uncertainty bounds as additional constraints
            constraints += [X[:, k+1] >= x_next - 0.1]  # Lower bound
            constraints += [X[:, k+1] <= x_next + 0.1]  # Upper bound
        
        # Control limits
        constraints += [cp.norm_inf(U) <= 1.0]  # Example control limits
        
        # Solve the optimization problem
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve()
        
        if problem.status not in ["infeasible", "unbounded"]:
            return U[:, 0].value  # Return first control action
        else:
            # Fallback to safe control if optimization fails
            return np.zeros(self.n_controls)
```

## Perception Transfer

### Domain Adaptation for Perception

Transferring perception models from simulation to reality:

```python
# Domain adaptation for perception models
import torch
import torch.nn as nn
import torch.nn.functional as F

class DomainAdaptationNetwork(nn.Module):
    def __init__(self, base_model, num_classes):
        super(DomainAdaptationNetwork, self).__init__()
        
        # Shared feature extractor (base model)
        self.feature_extractor = base_model
        
        # Task classifier (for the original task)
        self.classifier = nn.Linear(base_model.output_dim, num_classes)
        
        # Domain classifier (to distinguish sim vs real)
        self.domain_classifier = nn.Sequential(
            nn.Linear(base_model.output_dim, 100),
            nn.ReLU(),
            nn.Linear(100, 2)  # 2 domains: sim and real
        )
        
    def forward(self, x, lambda_val=0.5):
        # Extract features
        features = self.feature_extractor(x)
        
        # Task classification
        task_output = self.classifier(features)
        
        # Domain classification with gradient reversal
        reversed_features = GradientReversal(lambda_val)(features)
        domain_output = self.domain_classifier(reversed_features)
        
        return task_output, domain_output

class GradientReversal(torch.autograd.Function):
    """
    Gradient Reversal Layer
    """
    @staticmethod
    def forward(ctx, input, lambda_val):
        ctx.lambda_val = lambda_val
        return input
    
    @staticmethod
    def backward(ctx, grad_output):
        return -ctx.lambda_val * grad_output, None

# Training loop for domain adaptation
def train_domain_adaptation(model, source_loader, target_loader, optimizer, epochs=10):
    model.train()
    
    for epoch in range(epochs):
        for (source_data, source_labels), (target_data, _) in zip(source_loader, target_loader):
            # Set domain labels (0 for source/sim, 1 for target/real)
            source_domain_labels = torch.zeros(len(source_data)).long()
            target_domain_labels = torch.ones(len(target_data)).long()
            
            # Combine data
            combined_data = torch.cat([source_data, target_data], dim=0)
            combined_domains = torch.cat([source_domain_labels, target_domain_labels], dim=0)
            
            # Forward pass
            task_output, domain_output = model(combined_data)
            
            # Task loss on source data only
            task_loss = F.cross_entropy(
                task_output[:len(source_data)], 
                source_labels
            )
            
            # Domain loss on combined data
            domain_loss = F.cross_entropy(domain_output, combined_domains)
            
            # Total loss
            total_loss = task_loss + domain_loss
            
            # Backward pass
            optimizer.zero_grad()
            total_loss.backward()
            optimizer.step()
```

## Hardware-in-the-Loop Testing

### HIL Setup for Validation

Testing controllers in a hardware-in-the-loop environment before full deployment:

```python
# Hardware-in-the-loop testing
import threading
import time
import numpy as np

class HardwareInLoopTester:
    def __init__(self, sim_env, real_robot_interface):
        self.sim_env = sim_env
        self.real_robot = real_robot_interface
        self.is_running = False
        self.test_results = []
        
    def run_hil_test(self, policy, test_duration=60.0):
        """
        Run hardware-in-the-loop test where controller runs in sim
        but receives real robot state feedback
        """
        self.is_running = True
        
        # Start timing
        start_time = time.time()
        
        # Initialize state
        sim_state = self.sim_env.reset()
        real_state = self.real_robot.get_state()
        
        while time.time() - start_time < test_duration and self.is_running:
            # Apply control policy to real state (as if sim state)
            control_action = policy(real_state)
            
            # Execute action on real robot
            self.real_robot.send_command(control_action)
            
            # Update sim to match real robot state
            self.sim_env.update_state_from_real(real_state)
            
            # Get next state from real robot
            real_state = self.real_robot.get_state()
            
            # Record test metrics
            metrics = self.evaluate_performance(sim_state, real_state, control_action)
            self.test_results.append(metrics)
            
            # Small delay to match control frequency
            time.sleep(0.01)  # 100 Hz control frequency
        
        return self._analyze_results()
    
    def evaluate_performance(self, sim_state, real_state, action):
        # Evaluate how well sim and real states match
        state_difference = np.mean(np.abs(sim_state - real_state))
        
        # Evaluate control performance
        control_effort = np.mean(np.abs(action))
        
        # Evaluate stability metrics
        stability_metric = self._calculate_stability(real_state)
        
        return {
            'state_diff': state_difference,
            'control_effort': control_effort,
            'stability': stability_metric,
            'timestamp': time.time()
        }
    
    def _calculate_stability(self, state):
        # Calculate stability based on state (e.g., COM position, joint angles)
        # This is a simplified example
        com_height = state[2]  # Assuming COM height is at index 2
        base_orientation = state[3:7]  # Assuming quaternion is at indices 3-6
        
        # Stability is better when COM is high and orientation is upright
        stability = com_height * (1 - np.abs(base_orientation[2]))  # Simplified metric
        
        return stability
    
    def _analyze_results(self):
        # Analyze the collected HIL test results
        if not self.test_results:
            return {}
        
        # Extract metrics
        state_diffs = [r['state_diff'] for r in self.test_results]
        control_efforts = [r['control_effort'] for r in self.test_results]
        stabilities = [r['stability'] for r in self.test_results]
        
        analysis = {
            'avg_state_diff': np.mean(state_diffs),
            'std_state_diff': np.std(state_diffs),
            'avg_control_effort': np.mean(control_efforts),
            'avg_stability': np.mean(stabilities),
            'total_tests': len(self.test_results)
        }
        
        return analysis
```

## Deployment Strategies

### Gradual Deployment

Gradually transferring from simulation to reality:

```python
# Gradual deployment strategy
class GradualDeployment:
    def __init__(self, sim_env, real_robot):
        self.sim_env = sim_env
        self.real_robot = real_robot
        self.deployment_stage = 0  # 0: sim only, 1: sim+reality, 2: reality only
        self.confidence_threshold = 0.8
        self.performance_history = []
    
    def deploy_policy(self, policy):
        """
        Deploy policy with gradual transition from sim to real
        """
        while self.deployment_stage < 2:  # Until fully deployed on real robot
            if self.deployment_stage == 0:  # Pure simulation
                performance = self._test_in_simulation(policy)
                if performance > self.confidence_threshold:
                    self.deployment_stage = 1
                    print("Moving to mixed reality deployment")
                    
            elif self.deployment_stage == 1:  # Mixed reality
                performance = self._test_in_mixed_reality(policy)
                if performance > self.confidence_threshold:
                    self.deployment_stage = 2
                    print("Moving to full real robot deployment")
            
            # Record performance
            self.performance_history.append(performance)
            
            # Small delay between tests
            time.sleep(1.0)
    
    def _test_in_simulation(self, policy):
        # Run extensive tests in simulation
        total_reward = 0
        episodes = 100
        
        for _ in range(episodes):
            state = self.sim_env.reset()
            episode_reward = 0
            done = False
            
            while not done:
                action = policy(state)
                state, reward, done, _ = self.sim_env.step(action)
                episode_reward += reward
            
            total_reward += episode_reward
        
        avg_reward = total_reward / episodes
        return self._normalize_reward(avg_reward)
    
    def _test_in_mixed_reality(self, policy):
        # Test with real robot in safe environment
        # Use safety constraints and human supervision
        total_reward = 0
        trials = 20
        
        for _ in range(trials):
            # Reset robot to safe state
            self.real_robot.reset_to_safe_position()
            
            # Run policy with safety constraints
            state = self.real_robot.get_state()
            trial_reward = 0
            done = False
            steps = 0
            max_steps = 100
            
            while not done and steps < max_steps:
                action = policy(state)
                
                # Apply safety constraints
                safe_action = self._apply_safety_constraints(action)
                
                # Execute on real robot
                self.real_robot.send_command(safe_action)
                
                # Get next state
                next_state = self.real_robot.get_state()
                
                # Calculate reward
                reward = self._calculate_mixed_reality_reward(state, next_state, safe_action)
                
                state = next_state
                trial_reward += reward
                steps += 1
                
                # Check if robot is in unsafe state
                if self._is_unsafe_state(state):
                    done = True
            
            total_reward += trial_reward
        
        avg_reward = total_reward / trials
        return self._normalize_reward(avg_reward)
    
    def _apply_safety_constraints(self, action):
        # Apply limits to ensure safe operation
        # This might limit joint angles, velocities, etc.
        action = np.clip(action, -1.0, 1.0)  # Example limits
        return action
    
    def _calculate_mixed_reality_reward(self, state, next_state, action):
        # Calculate reward based on real robot state
        # This might be simpler than sim reward to prioritize safety
        reward = 0
        
        # Reward for staying in safe configuration
        if not self._is_unsafe_state(next_state):
            reward += 1.0
        
        # Small penalty for excessive control effort
        reward -= 0.01 * np.sum(np.square(action))
        
        return reward
    
    def _is_unsafe_state(self, state):
        # Check if robot is in an unsafe configuration
        # Example: joint limits, balance, etc.
        return False  # Simplified check
    
    def _normalize_reward(self, reward):
        # Normalize reward to [0, 1] range
        return min(1.0, max(0.0, reward / 100.0))  # Example normalization
```

## Performance Validation

### Validation Metrics

Key metrics for validating sim-to-real transfer:

```python
# Validation metrics for sim-to-real transfer
class TransferValidation:
    def __init__(self):
        self.metrics = {}
    
    def validate_transfer(self, policy, real_robot, sim_env, validation_tasks):
        """
        Validate sim-to-real transfer with multiple metrics
        """
        results = {}
        
        for task in validation_tasks:
            task_results = {}
            
            # Success rate
            task_results['success_rate'] = self._evaluate_success_rate(
                policy, real_robot, task
            )
            
            # Safety metrics
            task_results['safety_score'] = self._evaluate_safety_metrics(
                policy, real_robot, task
            )
            
            # Performance similarity
            task_results['similarity_score'] = self._evaluate_performance_similarity(
                policy, real_robot, sim_env, task
            )
            
            # Energy efficiency
            task_results['energy_efficiency'] = self._evaluate_energy_efficiency(
                policy, real_robot, task
            )
            
            # Execution time consistency
            task_results['timing_consistency'] = self._evaluate_timing_consistency(
                policy, real_robot, task
            )
            
            results[task] = task_results
        
        return results
    
    def _evaluate_success_rate(self, policy, real_robot, task):
        """
        Evaluate success rate of task execution
        """
        successes = 0
        total_attempts = 20
        
        for _ in range(total_attempts):
            success = self._run_task_and_check_success(policy, real_robot, task)
            if success:
                successes += 1
        
        return successes / total_attempts
    
    def _evaluate_safety_metrics(self, policy, real_robot, task):
        """
        Evaluate safety during task execution
        """
        safety_violations = 0
        total_executions = 10
        
        for _ in range(total_executions):
            violations = self._check_safety_violations(policy, real_robot, task)
            safety_violations += violations
        
        # Lower is better (fewer violations)
        avg_violations = safety_violations / total_executions
        
        # Convert to safety score (higher is better)
        safety_score = max(0.0, 1.0 - avg_violations)
        return safety_score
    
    def _evaluate_performance_similarity(self, policy, real_robot, sim_env, task):
        """
        Evaluate how similar the performance is between sim and real
        """
        # Run task in simulation
        sim_performance = self._execute_task_in_sim(sim_env, policy, task)
        
        # Run task on real robot
        real_performance = self._execute_task_on_real(real_robot, policy, task)
        
        # Calculate similarity (e.g., using correlation or normalized difference)
        similarity = self._calculate_performance_similarity(
            sim_performance, real_performance
        )
        
        return similarity
    
    def _evaluate_energy_efficiency(self, policy, real_robot, task):
        """
        Evaluate energy efficiency of the policy
        """
        total_energy = 0
        total_executions = 5
        
        for _ in range(total_executions):
            energy = self._measure_energy_consumption(policy, real_robot, task)
            total_energy += energy
        
        avg_energy = total_energy / total_executions
        
        # Convert to efficiency score (lower energy = higher efficiency)
        # Assuming 100J is a reference for normalization
        efficiency_score = max(0.0, min(1.0, 100.0 / (avg_energy + 1e-6)))
        return efficiency_score
    
    def _evaluate_timing_consistency(self, policy, real_robot, task):
        """
        Evaluate consistency of execution timing
        """
        execution_times = []
        total_executions = 10
        
        for _ in range(total_executions):
            exec_time = self._measure_execution_time(policy, real_robot, task)
            execution_times.append(exec_time)
        
        # Calculate consistency (inverse of coefficient of variation)
        mean_time = np.mean(execution_times)
        std_time = np.std(execution_times)
        
        if mean_time == 0:
            return 1.0  # Perfect consistency if no time variation
        
        # Coefficient of variation (lower is more consistent)
        cv = std_time / mean_time
        
        # Convert to consistency score (higher is better)
        consistency_score = max(0.0, min(1.0, 1.0 - cv))
        return consistency_score
```

## Summary

The sim-to-real bridge is a critical component of deploying AI-powered humanoid robots. Success requires:

1. **Understanding the reality gap**: Quantifying differences between simulation and reality
2. **Domain randomization**: Making models robust to variations
3. **System identification**: Calibrating simulation parameters to match reality
4. **Robust control design**: Controllers that work well in both domains
5. **Perception transfer**: Adapting computer vision models to real-world conditions
6. **Hardware-in-the-loop validation**: Testing before full deployment
7. **Gradual deployment**: Phased approach from simulation to reality
8. **Performance validation**: Quantitative metrics to ensure transfer success

The NVIDIA Isaac ecosystem provides comprehensive tools for each of these aspects, enabling the development of humanoid robots that perform effectively in real-world environments based on simulation-trained policies and perception models.