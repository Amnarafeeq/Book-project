import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From Digital Intelligence to Embodied Robotics',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-robotics-academy.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'your-organization', // Usually your GitHub org/user name.
  projectName: 'physical-ai-textbook', // Usually your repo name.

  onBrokenLinks: 'warn', // Changed from 'throw' to 'warn' to handle broken images
  onBrokenMarkdownLinks: 'warn',
  markdown: {
    mermaid: true,
    format: 'detect',
    parseFrontMatter: undefined,
  },

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
          sidebarPath: require.resolve('./sidebars.ts'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-organization/physical-ai-textbook/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: false, // Disable blog functionality
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig:
    {
      // Social card for the textbook
      image: 'img/logo.svg',
      markdown: {
        mermaid: true,
      },
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Robotics Academy Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorial',
            position: 'left',
            label: 'Modules',
          },
          {to: '/docs/overview', label: 'Course Overview', position: 'left'},
          {to: '/docs/hardware', label: 'Hardware', position: 'left'},
          {to: '/docs/capstone', label: 'Capstone', position: 'left'},
          {to: '/docs/glossary', label: 'Glossary', position: 'left'},
          {to: '/docs/resources', label: 'Resources', position: 'left'},
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Academy',
            items: [
              {
                label: 'Modules',
                to: '/docs/modules',
              },
              {
                label: 'Course Overview',
                to: '/docs/overview',
              },
              {
                label: 'Capstone Project',
                to: '/docs/capstone',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'Glossary',
                to: '/docs/glossary',
              },
              {
                label: 'Hardware Requirements',
                to: '/docs/hardware',
              },
              {
                label: 'GitHub Repository',
                href: 'https://github.com/your-organization/physical-ai-textbook',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/your-discord',
              },
              {
                label: 'Twitter',
                href: 'https://twitter.com/your-twitter',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Academy. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['bash', 'json', 'yaml', 'python', 'cpp', 'docker'],
      },
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
    },
};

export default config;
