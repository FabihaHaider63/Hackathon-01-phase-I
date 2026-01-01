---
title: Physics, Gravity, Collision Simulation
sidebar_position: 10
description: Understanding physics simulation in Gazebo for humanoid robots including gravity, collision detection, and realistic dynamics
---

# Physics, Gravity, Collision Simulation

## Overview

Physics simulation is crucial for creating realistic digital twins of humanoid robots. This chapter explores how Gazebo simulates physical interactions, including gravity, collision detection, and dynamic responses. Proper physics configuration ensures that simulated humanoid robots behave similarly to their real-world counterparts, which is essential for developing and testing control algorithms.

## Learning Objectives

By the end of this chapter, you should be able to:

- [ ] Understand how physics engines work in Gazebo
- [ ] Configure physics parameters for humanoid robots
- [ ] Set up realistic gravity and dynamics
- [ ] Implement collision detection and response
- [ ] Optimize physics parameters for performance
- [ ] Debug physics-related simulation issues
- [ ] Validate physics simulation against real-world behavior

## Physics Engines in Gazebo

### Overview of Available Physics Engines

Gazebo (now Ignition Gazebo) supports multiple physics engines:

1. **ODE (Open Dynamics Engine)**: Default engine, good for general-purpose simulation
2. **Bullet**: Provides better performance and more stable contacts
3. **SimBody**: From NASA, designed for articulated body systems
4. **DART (Dynamic Animation and Robotics Toolkit)**: Advanced physics with good stability

For humanoid robotics, ODE and Bullet are most commonly used.

### Physics Engine Configuration

Physics parameters are defined in the world file:

```xml
<physics name="humanoid_physics" type="ode">
  <!-- Simulation time step -->
  <max_step_size>0.001</max_step_size>
  
  <!-- Real-time factor (1.0 = real-time, >1.0 = faster than real-time) -->
  <real_time_factor>1.0</real_time_factor>
  
  <!-- Update rate (steps per second) -->
  <real_time_update_rate>1000</real_time_update_rate>
  
  <!-- Gravity (x, y, z components in m/s²) -->
  <gravity>0 0 -9.8</gravity>
  
  <!-- ODE-specific parameters -->
  <ode>
    <solver>
      <!-- Type of solver: "quick", "pgs", "dantzig" -->
      <type>quick</type>
      
      <!-- Number of iterations for the solver -->
      <iters>100</iters>
      
      <!-- Successive over-relaxation parameter -->
      <sor>1.3</sor>
    </solver>
    
    <constraints>
      <!-- Constraint Force Mixing: affects constraint stability -->
      <cfm>0.0</cfm>
      
      <!-- Error Reduction Parameter: affects constraint stability -->
      <erp>0.2</erp>
      
      <!-- Maximum velocity for contact correction -->
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      
      <!-- Contact surface layer (penetration allowed) -->
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### Choosing Physics Parameters for Humanoid Robots

For humanoid robots, the physics parameters are critical for stable simulation:

- **Time step**: 0.001s is typically needed for stable humanoid control
- **Update rate**: 1000 Hz to match real-time control rates
- **Solver iterations**: Higher values (100+) for better stability
- **CFM/ERP**: Adjust for desired constraint behavior (stability vs. rigidity)

## Gravity Simulation

### Understanding Gravity in Simulation

Gravity is a fundamental force in physics simulation that affects all objects with mass. For humanoid robots:

- Ensures feet stay in contact with the ground when standing
- Affects balance and posture control simulation
- Influences walking and locomotion patterns

### Configuring Gravity

```xml
<!-- Standard Earth gravity -->
<gravity>0 0 -9.8</gravity>

<!-- Different gravity for Moon simulation -->
<gravity>0 0 -1.62</gravity>

<!-- Zero gravity for space robotics -->
<gravity>0 0 0</gravity>

<!-- Custom gravity direction (e.g., for wall climbing robots) -->
<gravity>0 -9.8 0</gravity>
```

### Gravity Considerations for Humanoid Robots

For humanoid robot simulation, consider:

1. **Consistency**: Use the same gravity value in simulation and control algorithms
2. **Z-axis direction**: Standard is negative Z (downward in ROS coordinate frame)
3. **Units**: Always use m/s² for consistency with ROS units

## Collision Detection

### Collision Geometry vs. Visual Geometry

It's important to distinguish between collision and visual geometry:

- **Visual geometry**: How the robot looks (for rendering)
- **Collision geometry**: How the robot interacts physically (for physics simulation)

For performance, collision geometry is often simpler than visual geometry.

### Collision Shapes

Gazebo supports several primitive collision shapes:

```xml
<link name="collision_demo">
  <!-- Box collision -->
  <collision name="box_collision">
    <geometry>
      <box>
        <size>0.5 0.3 0.2</size>
      </box>
    </geometry>
  </collision>
  
  <!-- Cylinder collision -->
  <collision name="cylinder_collision">
    <geometry>
      <cylinder>
        <radius>0.1</radius>
        <length>0.5</length>
      </cylinder>
    </geometry>
  </collision>
  
  <!-- Sphere collision -->
  <collision name="sphere_collision">
    <geometry>
      <sphere>
        <radius>0.1</radius>
      </sphere>
    </geometry>
  </collision>
  
  <!-- Mesh collision -->
  <collision name="mesh_collision">
    <geometry>
      <mesh>
        <uri>package://humanoid_description/meshes/complex_shape.stl</uri>
        <!-- Optional scale -->
        <scale>1.0 1.0 1.0</scale>
      </mesh>
    </geometry>
  </collision>
</link>
```

### Collision Properties

For humanoid robots, collision properties are important for realistic simulation:

```xml
<gazebo reference="left_foot">
  <collision name="left_foot_collision">
    <!-- Surface properties -->
    <surface>
      <!-- Contact properties -->
      <contact>
        <ode>
          <!-- Spring stiffness -->
          <kp>1e+6</kp>
          
          <!-- Damping coefficient -->
          <kd>1e+3</kd>
          
          <!-- Number of contact points -->
          <max_vel>100.0</max_vel>
          
          <!-- Allowed penetration depth -->
          <min_depth>0.001</min_depth>
        </ode>
      </contact>
      
      <!-- Friction properties -->
      <friction>
        <ode>
          <!-- Primary friction value -->
          <mu>0.9</mu>
          
          <!-- Secondary friction value -->
          <mu2>0.9</mu2>
          
          <!-- Friction direction -->
          <fdir1>1 0 0</fdir1>
        </ode>
      </friction>
    </surface>
  </collision>
</gazebo>
```

### Collision Layers and Filtering

For humanoid robots with many body parts, you can control which parts collide:

```xml
<gazebo reference="right_upper_arm">
  <!-- Disable self-collision with specific links -->
  <self_collide>false</self_collide>
</gazebo>

<!-- Alternatively, use collision filtering -->
<link name="left_arm_base">
  <collision name="collision">
    <!-- Other collision elements -->
    <surface>
      <contact>
        <collide_without_contact>0</collide_without_contact>
      </contact>
    </surface>
  </collision>
</link>
```

## Dynamic Properties for Humanoid Robots

### Inertial Parameters

Accurate inertial properties are crucial for realistic dynamics:

```xml
<link name="left_thigh">
  <inertial>
    <!-- Mass in kg -->
    <mass value="3.0"/>
    
    <!-- Inertia matrix -->
    <!-- For a cylinder: 
         ixx = 1/12 * m * (3*r² + h²)
         iyy = 1/12 * m * (3*r² + h²) 
         izz = 1/2 * m * r² -->
    <inertia 
      ixx="0.05" 
      ixy="0.0" 
      ixz="0.0" 
      iyy="0.05" 
      iyz="0.0" 
      izz="0.01"/>
  </inertial>
</link>
```

### Calculating Inertial Properties

For common shapes:

- **Box** (width w, depth d, height h): 
  - ixx = 1/12 * m * (d² + h²)
  - iyy = 1/12 * m * (w² + h²)
  - izz = 1/12 * m * (w² + d²)

- **Cylinder** (radius r, height h):
  - ixx = 1/12 * m * (3*r² + h²)
  - iyy = 1/12 * m * (3*r² + h²)
  - izz = 1/2 * m * r²

- **Sphere** (radius r):
  - ixx = iyy = izz = 2/5 * m * r²

### Joint Dynamics

Joint dynamics affect how humanoid robots move:

```xml
<joint name="left_knee" type="revolute">
  <!-- Other joint properties -->
  
  <!-- Dynamics parameters -->
  <dynamics damping="1.0" friction="0.2"/>
</joint>
```

- **Damping**: Resistive force proportional to velocity (energy loss)
- **Friction**: Static force that must be overcome to start motion

## Contact Simulation for Humanoid Locomotion

### Foot-Ground Contact

For humanoid walking simulation, foot-ground contact is critical:

```xml
<gazebo reference="left_foot">
  <collision name="left_foot_collision">
    <surface>
      <contact>
        <ode>
          <!-- High stiffness for realistic foot contact -->
          <kp>1000000</kp>
          
          <!-- Damping for energy absorption -->
          <kd>1000</kd>
          
          <!-- Maximum velocity of contact correction -->
          <max_vel>100</max_vel>
          
          <!-- Minimum penetration depth -->
          <min_depth>0.001</min_depth>
        </ode>
      </contact>
      
      <!-- Friction for walking -->
      <friction>
        <ode>
          <mu>0.8</mu>  <!-- Primary friction -->
          <mu2>0.8</mu2> <!-- Secondary friction -->
          <!-- Friction direction (optional) -->
          <fdir1>1 0 0</fdir1>
        </ode>
      </friction>
    </surface>
  </collision>
</gazebo>
```

### Balance and Stability Considerations

For stable humanoid simulation:

1. **Foot size**: Make feet large enough to provide stable support
2. **Friction**: Set appropriate friction values (0.5-1.0 for rubber-like feet)
3. **Damping**: Proper damping in joints to prevent oscillation
4. **Time step**: Small enough time step (0.001s) for stable contacts

## Performance Optimization

### Physics Performance Considerations

For real-time humanoid simulation:

1. **Simplify collision geometry**: Use primitive shapes instead of complex meshes
2. **Reduce contact points**: Limit complex contact scenarios
3. **Optimize solver parameters**: Balance accuracy with performance
4. **Use appropriate update rates**: Match to controller requirements

### Multi-threaded Physics Simulation

Gazebo can utilize multiple threads for physics simulation:

```xml
<physics name="humanoid_physics" type="ode">
  <!-- Other physics parameters -->
  
  <!-- Enable threaded physics (if supported) -->
  <threaded>1</threaded>
</physics>
```

### Contact Parameters for Performance

```xml
<physics name="fast_physics" type="ode">
  <max_step_size>0.002</max_step_size>  <!-- Larger step for performance -->
  <real_time_factor>0.8</real_time_factor>  <!-- Slightly slower -->
  <real_time_update_rate>500</real_time_update_rate>  <!-- Lower update rate -->
  
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>  <!-- Fewer iterations -->
      <sor>1.3</sor>
    </solver>
  </ode>
</physics>
```

## Debugging Physics Issues

### Common Physics Problems in Humanoid Simulation

1. **Robot falls through ground**:
   - Check collision geometry for feet
   - Verify gravity direction and magnitude
   - Check if links have proper mass and inertia

2. **Excessive joint oscillation**:
   - Increase damping in joints
   - Use smaller time step
   - Adjust ERP/CFM values

3. **Robot sliding instead of walking**:
   - Increase friction coefficients
   - Check foot geometry
   - Verify contact parameters

4. **Instability during dynamic motions**:
   - Increase solver iterations
   - Check inertial parameters
   - Adjust contact stiffness

### Visualization Tools

Use Gazebo's visualization tools to debug physics:

```xml
<!-- Visualize contact points -->
<gazebo reference="left_foot">
  <contact>
    <max_contacts>10</max_contacts>
    <ode>
      <kp>1000000</kp>
      <kd>1000</kd>
    </ode>
  </contact>
  <!-- Contacts will be visualized in Gazebo GUI -->
</gazebo>
```

In Gazebo GUI, you can toggle:
- Contact visualization
- Collision shapes
- Center of mass visualization
- Inertia visualization

### Testing Physics Properties

Create a simple test to validate physics simulation:

```python
# Test script to validate humanoid physics
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped

class PhysicsValidator(Node):
    def __init__(self):
        super().__init__('physics_validator')
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # Timer to check stability
        self.timer = self.create_timer(1.0, self.check_stability)
        
        self.prev_positions = {}
        
    def joint_callback(self, msg):
        # Store current joint positions
        for i, name in enumerate(msg.name):
            if name in self.prev_positions:
                # Check if joint is moving unexpectedly
                if abs(msg.position[i] - self.prev_positions[name]) > 0.01:
                    self.get_logger().info(f'Unexpected movement in {name}')
            self.prev_positions[name] = msg.position[i]
    
    def check_stability(self):
        # Check if robot is stable (not falling over unexpectedly)
        # This is a simplified check - real validation would be more complex
        pass

def main(args=None):
    rclpy.init(args=args)
    node = PhysicsValidator()
    
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

## Digital Twin Validation

### Validating Simulation Physics

For digital twin applications, validate physics parameters by:

1. **Comparing real vs. simulated behavior**: Check that the simulated robot behaves similarly to the real one
2. **Calibrating parameters**: Adjust mass, inertia, friction values based on real robot data
3. **Validating sensor outputs**: Ensure simulated sensors provide similar data to real sensors
4. **Testing control algorithms**: Verify that controllers work in both simulation and reality

### Parameter Tuning Workflow

```bash
# 1. Start with CAD/computed values
# 2. Test basic stable poses in simulation
# 3. Compare with real robot behavior
# 4. Adjust parameters iteratively
# 5. Validate with multiple scenarios
```

## Advanced Physics Features

### Custom Physics Plugins

For humanoid robots with special requirements, you can create custom physics plugins:

```cpp
// Example physics plugin for humanoid-specific behavior
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <sdf/sdf.hh>

namespace ignition {
namespace gazebo {
namespace systems {

class HumanoidPhysics : public System, public ISystemConfigure {
 public:
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &_eventMgr) override {
    // Configure humanoid-specific physics behavior
  }
};

}  // namespace systems
}  // namespace gazebo
}  // namespace ignition
```

### Soft Body Simulation

For more realistic humanoid simulation, consider soft tissue effects:

```xml
<!-- Simplified approach: adjust dynamics parameters -->
<joint name="left_shoulder" type="revolute">
  <dynamics damping="2.0" friction="0.5" spring_reference="0.0" spring_stiffness="100.0"/>
</joint>
```

## Summary

Physics simulation is fundamental to creating effective digital twins for humanoid robots. Proper configuration of gravity, collision detection, and dynamic properties ensures that simulation results reliably reflect real-world behavior. For humanoid robots, special attention to foot-ground contact and balance-related parameters is essential for realistic locomotion simulation.

The parameters and techniques covered in this chapter form the foundation for stable, realistic humanoid robot simulation in Gazebo, enabling effective development and testing of control algorithms before deployment on physical hardware.

## Exercises

1. Create a simple humanoid model with accurate inertial parameters and test its stability in Gazebo.

2. Configure physics parameters for a stable walking simulation and verify the robot maintains balance.

3. Implement contact sensors on a humanoid robot's feet and validate the contact detection.

## Further Reading

- Gazebo Physics Documentation
- "Robotics: Modelling, Planning and Control" by Siciliano et al.
- "Springer Handbook of Robotics" - Chapter on Simulation
- Open Dynamics Engine (ODE) Documentation