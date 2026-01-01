import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/ai-book-docusaurus/__docusaurus/debug',
    component: ComponentCreator('/ai-book-docusaurus/__docusaurus/debug', 'b41'),
    exact: true
  },
  {
    path: '/ai-book-docusaurus/__docusaurus/debug/config',
    component: ComponentCreator('/ai-book-docusaurus/__docusaurus/debug/config', '40e'),
    exact: true
  },
  {
    path: '/ai-book-docusaurus/__docusaurus/debug/content',
    component: ComponentCreator('/ai-book-docusaurus/__docusaurus/debug/content', '82f'),
    exact: true
  },
  {
    path: '/ai-book-docusaurus/__docusaurus/debug/globalData',
    component: ComponentCreator('/ai-book-docusaurus/__docusaurus/debug/globalData', '4b2'),
    exact: true
  },
  {
    path: '/ai-book-docusaurus/__docusaurus/debug/metadata',
    component: ComponentCreator('/ai-book-docusaurus/__docusaurus/debug/metadata', '3d1'),
    exact: true
  },
  {
    path: '/ai-book-docusaurus/__docusaurus/debug/registry',
    component: ComponentCreator('/ai-book-docusaurus/__docusaurus/debug/registry', '73e'),
    exact: true
  },
  {
    path: '/ai-book-docusaurus/__docusaurus/debug/routes',
    component: ComponentCreator('/ai-book-docusaurus/__docusaurus/debug/routes', 'e28'),
    exact: true
  },
  {
    path: '/ai-book-docusaurus/docs',
    component: ComponentCreator('/ai-book-docusaurus/docs', '164'),
    routes: [
      {
        path: '/ai-book-docusaurus/docs',
        component: ComponentCreator('/ai-book-docusaurus/docs', '8f6'),
        routes: [
          {
            path: '/ai-book-docusaurus/docs',
            component: ComponentCreator('/ai-book-docusaurus/docs', '7a6'),
            routes: [
              {
                path: '/ai-book-docusaurus/docs/intro',
                component: ComponentCreator('/ai-book-docusaurus/docs/intro', 'f38'),
                exact: true
              },
              {
                path: '/ai-book-docusaurus/docs/module-1/chapter-outline',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-1/chapter-outline', '8b5'),
                exact: true
              },
              {
                path: '/ai-book-docusaurus/docs/module-1/intro',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-1/intro', '5e8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-1/nodes-topics-services',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-1/nodes-topics-services', '179'),
                exact: true
              },
              {
                path: '/ai-book-docusaurus/docs/module-1/python-ros-integration',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-1/python-ros-integration', 'f4e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-1/sensor-systems',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-1/sensor-systems', 'a5d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-1/urdf-humanoid',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-1/urdf-humanoid', '667'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-2/building-ros2-packages-python',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-2/building-ros2-packages-python', '47e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-2/gazebo-environment-setup',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-2/gazebo-environment-setup', '485'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-2/gazebo-simulation',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-2/gazebo-simulation', '7c5'),
                exact: true
              },
              {
                path: '/ai-book-docusaurus/docs/module-2/high-fidelity-rendering',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-2/high-fidelity-rendering', '0b0'),
                exact: true
              },
              {
                path: '/ai-book-docusaurus/docs/module-2/human-robot-interaction-unity',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-2/human-robot-interaction-unity', 'dc2'),
                exact: true
              },
              {
                path: '/ai-book-docusaurus/docs/module-2/intro',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-2/intro', '845'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-2/launch-files-parameters',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-2/launch-files-parameters', 'a27'),
                exact: true
              },
              {
                path: '/ai-book-docusaurus/docs/module-2/nodes-topics-services-actions',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-2/nodes-topics-services-actions', '288'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-2/physics-gravity-collision-simulation',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-2/physics-gravity-collision-simulation', 'c33'),
                exact: true
              },
              {
                path: '/ai-book-docusaurus/docs/module-2/ros2-architecture',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-2/ros2-architecture', '3f1'),
                exact: true
              },
              {
                path: '/ai-book-docusaurus/docs/module-2/ros2-fundamentals',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-2/ros2-fundamentals', 'c30'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-2/sensor-simulation',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-2/sensor-simulation', 'b83'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-2/unity-visualization',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-2/unity-visualization', '1b3'),
                exact: true
              },
              {
                path: '/ai-book-docusaurus/docs/module-2/urdf-sdf-formats',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-2/urdf-sdf-formats', '8c4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-3/cognitive-planning-actions',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-3/cognitive-planning-actions', '296'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-3/intro',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-3/intro', 'ae2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-3/isaac-environment-setup',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-3/isaac-environment-setup', '876'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-3/nvidia-isaac-architecture',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-3/nvidia-isaac-architecture', '78e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-3/perception-ai-models',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-3/perception-ai-models', '189'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-3/sim-to-real-bridge',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-3/sim-to-real-bridge', '72c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-3/training-loop',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-3/training-loop', '103'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-4/bipedal-locomotion-balance',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-4/bipedal-locomotion-balance', '6d0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-4/capstone-autonomous-humanoid',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-4/capstone-autonomous-humanoid', '89f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-4/cognitive-planning-llm-ros',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-4/cognitive-planning-llm-ros', '494'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-4/humanoid-kinematics-dynamics',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-4/humanoid-kinematics-dynamics', '038'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-4/manipulation-grasping',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-4/manipulation-grasping', '5ae'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-4/multi-modal-interaction',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-4/multi-modal-interaction', '708'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-4/vla-overview',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-4/vla-overview', '03d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book-docusaurus/docs/module-4/voice-to-action-whisper',
                component: ComponentCreator('/ai-book-docusaurus/docs/module-4/voice-to-action-whisper', '379'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/ai-book-docusaurus/',
    component: ComponentCreator('/ai-book-docusaurus/', '720'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
