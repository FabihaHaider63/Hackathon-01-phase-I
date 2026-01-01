// @ts-check
// `@type` JSDoc annotations allow IDEs and typecheckers to understand what
// we want to do. For this file, all interactions and all category
// instances are fully typed.

const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'AI-Driven Book on ROS 2',
  tagline: 'An educational resource on Physical AI and ROS 2',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://mr-computers.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it's usually '/<projectName>/'
  baseUrl: '/ai-book-docusaurus/',

  // GitHub pages deployment config.
  organizationName: 'mr-computers', // Usually your GitHub org/user name.
  projectName: 'ai-book-docusaurus', // Usually your repo name.

  // Even if you don't use GitHub pages, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
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
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-username/ai-book-docusaurus/edit/main/docs/',
        },
        blog: false, // Disable blog functionality
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Disable color mode toggle - only use light theme
      colorMode: {
        defaultMode: 'light',
        disableSwitch: true,
        respectPrefersColorScheme: false,
      },
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Textbook',
          },
        ],
      },
      // Footer is removed since we're using a custom footer in the page component
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
      },
    }),

  // GitHub Pages deployment settings
  trailingSlash: false,
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  // GitHub pages adds this automatically
  noIndex: false,
};

module.exports = config;