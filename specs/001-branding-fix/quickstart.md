# Quickstart Guide: Docusaurus Branding Fix

## Overview
This guide provides a step-by-step process to implement the branding fixes for the Physical AI & Humanoid Robotics textbook Docusaurus site. This includes fixing image rendering, updating the sidebar, replacing Docusaurus content with book-specific content, and customizing branding elements.

## Prerequisites
- Node.js 18+ installed
- npm or yarn package manager
- Access to Docusaurus project files in the `docusaurus/` directory

## Step-by-Step Implementation

### 1. Update Docusaurus Configuration
First, modify the `docusaurus.config.js` file to update brand-related settings:

```javascript
// docusaurus.config.js
module.exports = {
  // Site metadata
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive textbook on physical AI and humanoid robotics',
  url: 'https://your-book-domain.com',
  baseUrl: '/',
  organizationName: 'your-org', // Usually your GitHub org/user name
  projectName: 'physical-ai-humanoid-robotics-textbook', // Usually your repo name
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  
  // Favicon
  favicon: 'img/favicon.ico',
  
  // GitHub link configuration
  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo
          editUrl: 'https://github.com/your-username/your-book-repo/edit/main/',
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
          alt: 'Physical AI & Humanoid Robotics Logo',
          src: 'img/logo.svg', // Update with your logo path
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            href: 'https://github.com/your-username/your-book-repo', // Update with your repo link
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          // Add custom footer links if needed
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer/themes/github'),
        darkTheme: require('prism-react-renderer/themes/dracula'),
      },
    }),
};
```

### 2. Configure Sidebar Structure
Update the `sidebars.js` file to ensure the sidebar contains only the required modules in the correct order:

```javascript
// sidebars.js
/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1',
      items: [
        // Add specific pages for Module 1
        'module1/intro',
        'module1/topic1',
        'module1/topic2',
        // ... additional pages
      ],
    },
    {
      type: 'category',
      label: 'Module 2',
      items: [
        // Add specific pages for Module 2
        'module2/intro',
        'module2/topic1',
        'module2/topic2',
        // ... additional pages
      ],
    },
    {
      type: 'category',
      label: 'Module 3',
      items: [
        // Add specific pages for Module 3
        'module3/intro',
        'module3/topic1',
        'module3/topic2',
        // ... additional pages
      ],
    },
    {
      type: 'category',
      label: 'Module 4',
      items: [
        // Add specific pages for Module 4
        'module4/intro',
        'module4/topic1',
        'module4/topic2',
        // ... additional pages
      ],
    },
  ],
};

module.exports = sidebars;
```

### 3. Add Required Static Assets
Place your custom branding assets in the `static/img/` directory:
- `logo.svg` or `logo.png` - Your custom logo
- `favicon.ico` - Favicon for the site
- Any other images referenced in your documentation

### 4. Update Homepage Content
Replace the default Docusaurus homepage content in `src/pages/index.js` (or `index.tsx`) with book-specific content:

```jsx
// src/pages/index.js
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/module1/intro">
            Start Reading - Module 1
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="A comprehensive textbook on Physical AI & Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
```

### 5. Ensure Image Rendering
To fix image rendering issues, ensure all image references use the correct static asset path:

```markdown
<!-- In your markdown files, reference images like this -->
![Alternative text](/img/your-image.png)

<!-- Or if the image is in a subfolder -->
![Alternative text](/img/subfolder/your-image.png)
```

### 6. Accessibility Compliance
Ensure all images have appropriate alt text for WCAG 2.1 AA compliance:

```markdown
<!-- Good: Descriptive alt text -->
![Diagram showing the architecture of a humanoid robot with labeled components]

<!-- For decorative images -->
![Image showing background pattern](/img/decoration.png)
```

## Validation Steps

After implementing the changes, verify:

1. **Logo renders correctly**: Check that your logo appears in the navbar instead of alt text
2. **Images render correctly**: Navigate through all documentation pages to ensure images display properly
3. **Sidebar structure**: Verify that the sidebar contains only Module 1, Module 2, Module 3, Module 4 in the correct order
4. **Content branding**: Confirm that all content refers to the textbook rather than Docusaurus
5. **GitHub link**: Verify the GitHub link points to your textbook repository
6. **Footer content**: Ensure the footer contains only the book copyright information

## Performance Optimization

To meet the 3-second image load requirement:

1. Optimize images using tools like ImageOptim, TinyPNG, or similar
2. Use appropriate image formats (WebP for modern browsers with fallbacks)
3. Consider implementing lazy loading for images

## Troubleshooting

**Images showing alt text instead of rendering**:
- Check that files exist in the `static/img/` directory
- Verify correct file paths in your markdown
- Ensure file names don't contain special characters

**Sidebar showing extra items**:
- Review `sidebars.js` to ensure only required modules are listed
- Check for auto-generated categories that may need to be removed

**Docusaurus branding still visible**:
- Double-check `docusaurus.config.js` values for title, favicon, etc.
- Look for any remaining Docusaurus-specific content in your markdown files