import { DefaultSidebarItemCategory } from '@docusaurus/plugin-content-docs';

const sidebars = {
  tutorialSidebar: [
    'intro', // main intro page
    {
      type: 'category',
      label: 'Capstone Autonomous Humanoid',
      items: [
        'capstone-autonomous-humanoid/index',
        'capstone-autonomous-humanoid/humanoid-design',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS2',
      items: [
        'module-1-ros2/index',
        'module-1-ros2/architecture-concepts',
        'module-1-ros2/communication-patterns',
        'module-1-ros2/nodes-packages-workspaces',
        'module-1-ros2/ros2-simulation',
        'module-1-ros2/tools-debugging',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation',
      items: [
        'module-2-simulation/index',
        'module-2-simulation/gazebo-fundamentals',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Isaac',
      items: [
        'module-3-isaac/index',
        'module-3-isaac/isaac-overview',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA Systems',
      items: [
        'module-4-vla-systems/index',
        'module-4-vla-systems/vla-architecture',
      ],
    },
    {
      type: 'category',
      label: 'Tutorial Basics',
      items: [
        'tutorial-basics/congratulations',
        'tutorial-basics/create-a-blog-post',
        'tutorial-basics/create-a-document',
        'tutorial-basics/create-a-page',
        'tutorial-basics/deploy-your-site',
        'tutorial-basics/markdown-features',
      ],
    },
    {
      type: 'category',
      label: 'Tutorial Extras',
      items: [
        'tutorial-extras/manage-docs-versions',
        'tutorial-extras/translate-your-site',
      ],
    },
  ],
};

export default sidebars;
