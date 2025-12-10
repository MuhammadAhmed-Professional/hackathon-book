/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // Physical AI & Humanoid Robotics Textbook Navigation
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction to Physical AI',
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 - The Robotic Nervous System',
      collapsed: true,
      items: [
        'module1/module1-index',
        'module1/ros2-architecture',
        'module1/nodes-topics-services',
        'module1/python-integration',
        'module1/actions-and-services',
        'module1/parameters',
        'module1/urdf-for-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Gazebo & Unity - The Digital Twin',
      collapsed: true,
      items: [
        'module2/module2-index',
        'module2/gazebo-simulation',
        'module2/urdf-vs-sdf',
        'module2/physics-simulation',
        'module2/unity-rendering',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac - The AI-Robot Brain',
      collapsed: true,
      items: [
        'module3/module3-index',
        'module3/isaac-sim',
        'module3/isaac-ros',
        'module3/jetson-deployment',
        'module3/performance-optimization',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      collapsed: true,
      items: [
        'module4/README',
        // Temporarily disabled - files were moved to .bak due to build errors
        // 'module4/module4-index',
        // 'module4/sensor-fusion',
        // 'module4/motion-planning',
        // 'module4/control-systems',
        // 'module4/computer-vision',
        // 'module4/capstone',
      ],
    },
    {
      type: 'doc',
      id: 'hardware',
      label: 'Hardware Requirements',
    },
    {
      type: 'doc',
      id: 'weekly-breakdown',
      label: '13-Week Course Schedule',
    },
  ],
};

module.exports = sidebars;

