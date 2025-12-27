// @ts-check
// `@ts-check` enables tsconfig.json's "skipLibCheck" to reduce type-checking overhead

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics: A Technical Textbook',
  tagline: 'Comprehensive educational resource for Physical AI & Humanoid Robotics',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://physical-ai-humanoid-robotics.github.io',
  // Set the /<base>/ pathname under which your site is served
  // For GitHub Pages, this is usually /<project-name>/
  baseUrl: '/physical-ai-humanoid-robotics/',

  // GitHub pages deployment config.
  organizationName: 'physical-ai-humanoid-robotics', // Usually your GitHub org/user name.
  projectName: 'physical-ai-humanoid-robotics', // Usually your repo name.

  onBrokenLinks: 'warn',
  markdown: {
    mermaid: true,
    hooks: {
      onBrokenMarkdownLinks: 'warn',
      onBrokenMarkdownImages: 'warn',
    },
  },

  // Performance and image loading optimization
  themes: [
    '@docusaurus/theme-classic',
    '@docusaurus/theme-search-algolia',
  ],

  plugins: [
    // Performance optimization plugin
    [
      '@docusaurus/plugin-content-docs',
      {
        showLastUpdateTime: true,
        showLastUpdateAuthor: true,
      },
    ],
    // Image optimization plugin
    [
      '@docusaurus/plugin-ideal-image',
      {
        quality: 80,
        max: 1030, // max resized image's size.
        min: 640, // min resized image's size. if original is lower, use that size.
        steps: 2, // the max number of images generated between min and max (inclusive)
        disableInDev: false,
      },
    ],
  ],

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'es', 'fr', 'ur'], // Added locales for multi-language support
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Edit this page links removed for academic textbook
        },
        blog: false, // Optional: disable the blog plugin
        theme: {
          customCss: [
            require.resolve('./src/css/custom.css'),
            require.resolve('./src/css/academic-theme.css'), // Added academic theme
          ],
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/textbook-social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Physical AI & Humanoid Robotics Textbook Logo',
          src: 'img/book-logo.svg', // Updated to book-themed logo
          srcDark: 'img/book-logo-dark.svg', // Added dark mode logo
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'textbookSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            type: 'localeDropdown', // Added locale dropdown for multi-language support
            position: 'right',
          },
          {
            href: 'https://github.com/Areeshfatima/Physical-AI-Humanoid-Robotics-Book',
            label: 'GitHub',
            position: 'right',
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
                to: '/docs/intro',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/Areeshfatima/Physical-AI-Humanoid-Robotics-Book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. All rights reserved.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
        additionalLanguages: ['python', 'bash', 'cmake'],
      },
    }),
};

module.exports = config;