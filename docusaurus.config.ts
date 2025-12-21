import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'An AI-Native Interactive Textbook for Teaching Physical AI and Humanoid Robotics',
  favicon: 'img/favicon.ico',

  // Custom fields for API configuration
  // API_URL must be set in Vercel environment variables
  customFields: {
    apiUrl: process.env.API_URL || 'http://localhost:8000',
  },

  // Custom plugin to proxy API requests to backend during development
  plugins: [
    function proxyPlugin() {
      return {
        name: 'api-proxy-plugin',
        configureWebpack() {
          return {
            devServer: {
              proxy: [
                {
                  context: ['/api'],
                  target: 'http://localhost:8000',
                  changeOrigin: true,
                  secure: false,
                },
              ],
            },
          };
        },
      };
    },
  ],

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://shery-ai-robotics-book.vercel.app',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For Vercel deployment, use root path
  baseUrl: '/',

  // GitHub pages deployment config.
  organizationName: 'shery123pk', // Usually your GitHub org/user name.
  projectName: 'shery_ai_robotics_book', // Usually your repo name.

  onBrokenLinks: 'warn', // Temporarily set to warn until we create all module content

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: 'docs', // Serve docs at /docs
          editUrl: 'https://github.com/shery123pk/shery_ai_robotics_book/tree/main/',
        },
        blog: false, // Disable blog functionality
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Robotics',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {
          type: 'dropdown',
          label: 'Modules',
          position: 'left',
          items: [
            {
              label: 'Module 1: ROS 2',
              to: '/docs/module-1/ros2-intro',
            },
            {
              label: 'Module 2: Gazebo & Unity',
              to: '/docs/module-2/gazebo-intro',
            },
            {
              label: 'Module 3: NVIDIA Isaac',
              to: '/docs/module-3/isaac-sim-intro',
            },
            {
              label: 'Module 4: VLA',
              to: '/docs/module-4/vla-intro',
            },
          ],
        },
        {
          href: 'https://github.com/shery123pk/shery_ai_robotics_book',
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
              to: '/docs/module-1/ros2-intro',
            },
            {
              label: 'Module 2: Gazebo & Unity',
              to: '/docs/module-2/gazebo-intro',
            },
            {
              label: 'Module 3: NVIDIA Isaac',
              to: '/docs/module-3/isaac-sim-intro',
            },
            {
              label: 'Module 4: VLA',
              to: '/docs/module-4/vla-intro',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Hardware Requirements',
              to: '/docs/hardware-requirements',
            },
            {
              label: 'GitHub Repository',
              href: 'https://github.com/shery123pk/shery_ai_robotics_book',
            },
          ],
        },
        {
          title: 'About',
          items: [
            {
              label: 'Panaversity',
              href: 'https://panaversity.org',
            },
            {
              label: 'AI-Native Textbooks',
              href: 'https://ai-native.panaversity.org',
            },
          ],
        },
      ],
      copyright: `© ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Claude Code and Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
