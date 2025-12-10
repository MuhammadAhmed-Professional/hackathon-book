// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: '13-Week Course on Embodied Intelligence - From ROS 2 to Vision-Language-Action',
  favicon: 'img/favicon.ico',

  // GitHub Pages configuration
  url: 'https://MuhammadAhmed-Professional.github.io', // GitHub username
  baseUrl: '/hackathon-book/', // Repository name
  organizationName: 'MuhammadAhmed-Professional', // GitHub username
  projectName: 'hackathon-book', // Repository name
  deploymentBranch: 'gh-pages', // Branch for deployment

  onBrokenLinks: 'throw',
  trailingSlash: false, // Explicit trailingSlash for GitHub Pages
  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Remove this to remove the "edit this page" links.
          editUrl: 'https://github.com/MuhammadAhmed-Professional/hackathon-book/tree/master/frontend/',
          // Temporarily exclude Module 4 files with math expression errors
          exclude: [
            '**/module4/sensor-fusion.md',
            '**/module4/motion-planning.md',
            '**/module4/control-systems.md',
            '**/module4/computer-vision.md',
          ],
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/physical-ai-social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            to: '/signin',
            label: 'Sign In',
            position: 'right',
          },
          {
            to: '/signup',
            label: 'Sign Up',
            position: 'right',
          },
          {
            href: 'https://github.com/MuhammadAhmed-Professional/hackathon-book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Course Modules',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro',
              },
              {
                label: 'Module 1: ROS 2',
                to: '/docs/module1',
              },
              {
                label: 'Module 2: Gazebo & Unity',
                to: '/docs/module2',
              },
              {
                label: 'Module 3: NVIDIA Isaac',
                to: '/docs/module3',
              },
              {
                label: 'Module 4: VLA',
                to: '/docs/module4',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'Hardware Requirements',
                to: '/docs/hardware',
              },
              {
                label: 'Weekly Breakdown',
                to: '/docs/weekly-breakdown',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/MuhammadAhmed-Professional/hackathon-book',
              },
              {
                label: 'Hackathon Submission',
                href: 'https://github.com/MuhammadAhmed-Professional/hackathon-book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Course. Built for Panaversity Hackathon with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['python', 'bash', 'yaml', 'cpp', 'cmake'],
      },
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
      metadata: [
        {name: 'keywords', content: 'physical ai, humanoid robotics, ros 2, gazebo, nvidia isaac, vision-language-action, robotics course, embodied intelligence'},
        {name: 'description', content: 'Comprehensive 13-week textbook covering Physical AI & Humanoid Robotics: ROS 2, Gazebo/Unity simulation, NVIDIA Isaac platform, and Vision-Language-Action systems.'},
      ],
    }),
};

module.exports = config;

