// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer/themes/github');
const darkCodeTheme = require('prism-react-renderer/themes/dracula');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'Bridging the gap between the digital brain and the physical body.',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://ai-driven-development.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/AI-driven-development/',

  // GitHub pages deployment config.
  organizationName: 'ai-driven-development', // Usually your GitHub org/user name.
  projectName: 'AI_Robotics_book', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

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
          editUrl: 'https://github.com/ai-driven-development/AI_Robotics_book/tree/main/',
        },
        blog: {
          showReadingTime: true,
          editUrl: 'https://github.com/ai-driven-development/AI_Robotics_book/tree/main/',
        },
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
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'My Site Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            to: '/AI-driven-development/docs/01-intro-to-ros2',
            position: 'left',
            label: 'Textbook',
          },
          {
            label: 'Modules',
            position: 'left',
            items: [
              {
                label: 'Module 1: ROS 2',
                to: '/AI-driven-development/docs/01-intro-to-ros2',
              },
              {
                label: 'Module 2: Digital Twin',
                to: '/AI-driven-development/docs/04-urdf-for-humanoids',
              },
              {
                label: 'Module 3: AI-Robot Brain',
                to: '/AI-driven-development/docs/08-isaac-sim-intro',
              },
              {
                label: 'Module 4: VLA',
                to: '/AI-driven-development/docs/11-vla-whisper',
              },
            ]
          },
          {
            label: 'Contact',
            position: 'right',
            items: [
              {
                label: 'GitHub (repo)',
                href: 'https://github.com/MUKARRAM-ONE/AI-driven-development',
              },
              {
                label: 'LinkedIn',
                href: 'https://www.linkedin.com/in/mukarram-razzaq-0146572ba/',
              },
            ],
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Textbook',
            items: [
              {
                label: 'Introduction',
                to: '/AI-driven-development/docs/01-intro-to-ros2',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/MUKARRAM-ONE/AI-driven-development/tree/main/AI_Robotics_book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Created by Mukarram Razzaq Using AI Spec Driven Development. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
      },
    }),
};

module.exports = config;