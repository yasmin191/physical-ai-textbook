import { themes as prismThemes } from "prism-react-renderer";
import type { Config } from "@docusaurus/types";
import type * as Preset from "@docusaurus/preset-classic";

const config: Config = {
  title: "Physical AI & Humanoid Robotics",
  tagline:
    "From Digital Brain to Physical Body - A Comprehensive Course Textbook",
  favicon: "img/favicon.ico",

  future: {
    v4: true,
  },

  // GitHub Pages deployment
  url: "https://yasmin191.github.io",
  baseUrl: "/physical-ai-textbook/",

  organizationName: "yasmin191",
  projectName: "physical-ai-textbook",
  trailingSlash: false,
  deploymentBranch: "gh-pages",

  onBrokenLinks: "throw",
  onBrokenMarkdownLinks: "warn",

  i18n: {
    defaultLocale: "en",
    locales: ["en", "ur"],
    localeConfigs: {
      en: {
        label: "English",
        direction: "ltr",
      },
      ur: {
        label: "اردو",
        direction: "rtl",
      },
    },
  },

  presets: [
    [
      "classic",
      {
        docs: {
          sidebarPath: "./sidebars.ts",
          routeBasePath: "/",
          editUrl:
            "https://github.com/panaversity/hackathon_1/tree/main/textbook/",
        },
        blog: false,
        theme: {
          customCss: "./src/css/custom.css",
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: "img/physical-ai-social-card.jpg",
    colorMode: {
      defaultMode: "light",
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: "Physical AI & Robotics",
      logo: {
        alt: "Physical AI Logo",
        src: "img/logo.svg",
      },
      items: [
        {
          type: "docSidebar",
          sidebarId: "textbookSidebar",
          position: "left",
          label: "Textbook",
        },
        {
          type: "localeDropdown",
          position: "right",
        },
        {
          href: "https://github.com/panaversity/hackathon_1",
          label: "GitHub",
          position: "right",
        },
      ],
    },
    footer: {
      style: "dark",
      links: [
        {
          title: "Course Modules",
          items: [
            {
              label: "Module 1: ROS 2",
              to: "/module-1-ros2/intro-physical-ai",
            },
            {
              label: "Module 2: Simulation",
              to: "/module-2-simulation/gazebo-setup",
            },
            {
              label: "Module 3: NVIDIA Isaac",
              to: "/module-3-isaac/isaac-overview",
            },
            {
              label: "Module 4: VLA",
              to: "/module-4-vla/kinematics-dynamics",
            },
          ],
        },
        {
          title: "Resources",
          items: [
            { label: "Appendices", to: "/appendices/a-hardware-setup" },
            { label: "Panaversity", href: "https://panaversity.org" },
          ],
        },
        {
          title: "Community",
          items: [
            {
              label: "GitHub",
              href: "https://github.com/panaversity/hackathon_1",
            },
            { label: "PIAIC", href: "https://piaic.org" },
            { label: "GIAIC", href: "https://giaic.org" },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Panaversity. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ["python", "bash", "yaml", "json", "markup"],
    },
    docs: {
      sidebar: {
        hideable: true,
        autoCollapseCategories: true,
      },
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
