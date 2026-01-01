// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Physical AI & Humanoid Robotics Textbook',
      items: [
        {
          type: 'category',
          label: 'Module 1: Weeks 1-2: Introduction to Physical AI',
          items: [
            'module-1/intro',  // Introduction to Physical AI
            'module-1/urdf-humanoid',  // Humanoid Robotics Overview (mapping to URDF content)
            'module-1/python-ros-integration',  // Physical AI and Embodied Intelligence (mapping to Python integration)
            'module-1/sensor-systems'  // Sensors in Physical AI Systems
          ],
          collapsed: false,
        },
        {
          type: 'category',
          label: 'Module 2: Weeks 3-7: ROS 2 Fundamentals & Robot Simulation with Gazebo',
          items: [
            'module-2/intro',  // Introduction to the Digital Twin
            'module-2/urdf-sdf-formats',  // Humanoid URDF - Links, Joints, Sensors
            'module-2/ros2-fundamentals',  // Robot Data Flow - Sensor→Processing→Actuation
            'module-2/nodes-topics-services-actions',  // ROS 2 Architecture - Nodes, Topics, Services, Actions
            'module-2/building-ros2-packages-python',  // ROS 2 with Python - rclpy Usage
            'module-2/gazebo-environment-setup',  // Gazebo Physics, Collisions, Environment Design
            'module-2/sensor-simulation'  // Sensor Simulation (LiDAR, Depth, IMU)
          ],
          collapsed: false,
        },
        {
          type: 'category',
          label: 'Module 3: Weeks 8-10: The AI-Robot Brain (NVIDIA Isaac™)',
          items: [
            'module-3/intro',  // AI-Robot Brain Overview
            'module-3/nvidia-isaac-architecture',  // NVIDIA Isaac Architecture
            'module-3/cognitive-planning-actions',  // Cognitive Planning & Actions
            'module-3/isaac-environment-setup',  // Isaac Environment Setup
            'module-3/perception-ai-models',  // Perception & AI Models
            'module-3/training-loop',  // Training Loop
            'module-3/sim-to-real-bridge'  // Sim-to-Real Bridge
          ],
          collapsed: false,
        },
        {
          type: 'category',
          label: 'Module 4: Weeks 11-13: Vision-Language-Action (VLA)',
          items: [
            'module-4/vla-overview',
            'module-4/voice-to-action-whisper',
            'module-4/cognitive-planning-llm-ros',
            'module-4/humanoid-kinematics-dynamics',
            'module-4/bipedal-locomotion-balance',
            'module-4/manipulation-grasping',
            'module-4/multi-modal-interaction',
            'module-4/capstone-autonomous-humanoid'
          ],
          collapsed: false,
        }
      ],
      collapsed: false,
    },
  ],
};

module.exports = sidebars;