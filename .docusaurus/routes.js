import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', 'c16'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', 'd95'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'fe2'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', '832'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', 'f40'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '909'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', 'c94'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '55c'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'f76'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', 'd2e'),
            routes: [
              {
                path: '/docs/',
                component: ComponentCreator('/docs/', 'a8c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/',
                component: ComponentCreator('/docs/digital-twin/', '2da'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/gazebo-physics',
                component: ComponentCreator('/docs/digital-twin/gazebo-physics', '725'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/sensor-simulation',
                component: ComponentCreator('/docs/digital-twin/sensor-simulation', 'cd5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/unity-digital-twin',
                component: ComponentCreator('/docs/digital-twin/unity-digital-twin', 'e38'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-sim/',
                component: ComponentCreator('/docs/isaac-sim/', '0f7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-sim/',
                component: ComponentCreator('/docs/isaac-sim/', 'b11'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-sim/isaac-ros',
                component: ComponentCreator('/docs/isaac-sim/isaac-ros', '497'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-sim/nav2-humanoid',
                component: ComponentCreator('/docs/isaac-sim/nav2-humanoid', '7ca'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-foundations/',
                component: ComponentCreator('/docs/ros2-foundations/', 'ce7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-foundations/bridging-ai-robots',
                component: ComponentCreator('/docs/ros2-foundations/bridging-ai-robots', 'a0d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-foundations/communication',
                component: ComponentCreator('/docs/ros2-foundations/communication', '5bd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-foundations/foundations',
                component: ComponentCreator('/docs/ros2-foundations/foundations', 'ed7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla-system/',
                component: ComponentCreator('/docs/vla-system/', 'c7c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla-system/autonomous-humanoid',
                component: ComponentCreator('/docs/vla-system/autonomous-humanoid', 'e6a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla-system/cognitive-planning',
                component: ComponentCreator('/docs/vla-system/cognitive-planning', '9bb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla-system/quality-checklist',
                component: ComponentCreator('/docs/vla-system/quality-checklist', '841'),
                exact: true
              },
              {
                path: '/docs/vla-system/voice-to-action',
                component: ComponentCreator('/docs/vla-system/voice-to-action', '964'),
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
    path: '*',
    component: ComponentCreator('*'),
  },
];
