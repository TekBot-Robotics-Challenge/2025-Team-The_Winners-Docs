// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking

import {themes as prismThemes} from 'prism-react-renderer';

// Configuration Docusaurus
/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'The Winners',
  tagline: 'TEKBOT ROBOTICS Challenge',
  favicon: 'img/fav.ico',

  onBrokenLinks: 'ignore',
  onBrokenMarkdownLinks: 'warn',

  // Activation des options futures (préparation Docusaurus v4)
  future: {
    v4: true,
  },

  // Adresse de production du site
  url: 'https://TekBot-Robotics-Challenge.github.io',
  baseUrl: '/2025-Team-The_Winners-Docs/',

  // Configuration GitHub Pages
  organizationName: 'TekBot-Robotics-Challenge',
  projectName: '2025-Team-The_Winners-Docs',

  // Langue du site
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
          sidebarPath: './sidebars.js',
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Home',
        logo: {
          alt: 'The Winners',
          src: 'img/logo.png',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'aboutSidebar',
            position: 'left',
            label: 'About',
          },
          {
            type: 'docSidebar',
            sidebarId: 'preselectionSidebar',
            position: 'left',
            label: 'Pre-selection Tests',
          },
          {
            type: 'docSidebar',
            sidebarId: 'robotDesignSidebar',
            position: 'left',
            label: 'Robot Design',
          },
          {
            type: 'docSidebar',
            sidebarId: 'mediaSidebar',
            position: 'left',
            label: 'Media',
          },
          {
            type: 'docSidebar',
            sidebarId: 'linkSidebar',
            position: 'left',
            label: 'Links',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Tutorial',
                to: '/docs/tutorial',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions',
              },
              {
                label: 'Discord',
                href: 'https://discord.com/',
              },
              {
                label: 'X',
                href: 'https://x.com/',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} My Project, Inc. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;
