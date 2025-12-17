// Module 1: The Robotic Nervous System (ROS 2) - Sidebar Configuration
// This configuration should be integrated into the main Docusaurus sidebars.js file

module.exports = {
  module1Sidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros2/index',
        {
          type: 'category',
          label: 'Chapters',
          items: [
            'module-1-ros2/01-overview-architecture',
            'module-1-ros2/02-nodes-topics-services',
            'module-1-ros2/03-python-rclpy-control',
            'module-1-ros2/04-urdf-basics',
          ],
        },
      ],
    },
  ],
};

// To integrate into main sidebars.js:
// 1. Copy the module1Sidebar structure
// 2. Add to the appropriate section in your Docusaurus sidebars.js
// 3. Ensure all referenced MDX files exist in docs/module-1-ros2/
