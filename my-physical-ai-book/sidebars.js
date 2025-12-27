module.exports = {
  docs: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'ROS 2: Robotic Nervous System',
      items: [
        'ros2/intro',
        'ros2/nodes-topics-services',
        'ros2/rclpy-agent',
        'ros2/urdf-humanoid'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Digital Twin Simulation',
      items: [
        'digital-twin/intro',
        'digital-twin/gazebo',
        'digital-twin/unity',
        'digital-twin/sensors'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'AI-Robot Brain',
      items: [
        'ai-brain/intro',
        'ai-brain/isaac-sim',
        'ai-brain/isaac-ros',
        'ai-brain/nav2'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Vision-Language-Action (VLA)',
      items: [
        'vla/intro',
        'vla/voice-to-action',
        'vla/llm-planning'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone/autonomous-humanoid'
      ],
      collapsed: false,
    }
  ],
};