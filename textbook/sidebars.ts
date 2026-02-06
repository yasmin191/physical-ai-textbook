import type { SidebarsConfig } from "@docusaurus/plugin-content-docs";

const sidebars: SidebarsConfig = {
  textbookSidebar: [
    "intro",
    {
      type: "category",
      label: "Module 1: ROS 2 Fundamentals",
      collapsed: false,
      items: [
        "module-1-ros2/intro-physical-ai",
        "module-1-ros2/humanoid-landscape",
        "module-1-ros2/sensor-systems",
        "module-1-ros2/ros2-architecture",
        "module-1-ros2/nodes-topics-services",
        "module-1-ros2/ros2-python-packages",
        "module-1-ros2/launch-files",
        "module-1-ros2/urdf",
      ],
    },
    {
      type: "category",
      label: "Module 2: Simulation",
      collapsed: true,
      items: [
        "module-2-simulation/gazebo-setup",
        "module-2-simulation/urdf-sdf",
        "module-2-simulation/physics-simulation",
        "module-2-simulation/sensor-simulation",
        "module-2-simulation/unity-simulation",
      ],
    },
    {
      type: "category",
      label: "Module 3: NVIDIA Isaac",
      collapsed: true,
      items: [
        "module-3-isaac/isaac-overview",
        "module-3-isaac/isaac-sim",
        "module-3-isaac/isaac-ros",
        "module-3-isaac/visual-slam",
        "module-3-isaac/nav2",
        "module-3-isaac/reinforcement-learning",
        "module-3-isaac/sim-to-real",
      ],
    },
    {
      type: "category",
      label: "Module 4: Vision-Language-Action",
      collapsed: true,
      items: [
        "module-4-vla/kinematics-dynamics",
        "module-4-vla/bipedal-locomotion",
        "module-4-vla/manipulation",
        "module-4-vla/voice-to-action",
        "module-4-vla/llm-planning",
        "module-4-vla/conversational-robotics",
        "module-4-vla/multimodal-interaction",
        "module-4-vla/capstone",
      ],
    },
    {
      type: "category",
      label: "Appendices",
      collapsed: true,
      items: [
        "appendices/a-hardware-setup",
        "appendices/b-ros2-installation",
        "appendices/c-isaac-setup",
        "appendices/d-jetson-setup",
        "appendices/e-math-foundations",
      ],
    },
  ],
};

export default sidebars;
