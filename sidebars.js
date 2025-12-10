// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'index',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Foundations',
      items: [
        'ros2-foundations/index',
        'ros2-foundations/foundations',
        'ros2-foundations/communication',
        'ros2-foundations/bridging-ai-robots',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin Simulation',
      items: [
        'digital-twin/index',
        'digital-twin/gazebo-physics',
        'digital-twin/unity-digital-twin',
        'digital-twin/sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'isaac-sim/index',
        'isaac-sim/isaac-sim',
        'isaac-sim/isaac-ros',
        'isaac-sim/nav2-humanoid',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) System',
      items: [
        'vla-system/index',
        'vla-system/voice-to-action',
        'vla-system/cognitive-planning',
        'vla-system/autonomous-humanoid',
      ],
    },
  ],
};

module.exports = sidebars;