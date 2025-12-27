# CSS Override Rules

## Overview
This document defines the custom CSS overrides needed to replace Docusaurus default styling with book-specific academic styling, while maintaining the content structure and functionality.

## Color Palette Overrides
```css
/* Academic Color Scheme */
:root {
  /* Primary colors for academic theme */
  --ifm-color-primary: #1a365d; /* Deep navy blue for headers and accents */
  --ifm-color-primary-dark: #132943; /* Darker variant */
  --ifm-color-primary-darker: #0f2036; /* Even darker variant */
  --ifm-color-primary-darkest: #0a1524; /* Darkest variant */
  --ifm-color-primary-light: #2a4a70; /* Lighter variant */
  --ifm-color-primary-lighter: #3a5f88; /* Even lighter variant */
  --ifm-color-primary-lightest: #4d739f; /* Lightest variant */

  /* Backgrounds */
  --ifm-background-color: #ffffff; /* Clean white background */
  --ifm-background-surface-color: #f8f9fa; /* Light gray for surfaces */
  --ifm-background-surface-color-dark: #e9ecef; /* Darker surface variant */

  /* Text colors */
  --ifm-font-color-base: #212529; /* Dark gray for body text (for eye comfort) */
  --ifm-font-color-base-inverse: #ffffff; /* White for light backgrounds */
  --ifm-heading-color: #1a365d; /* Match primary color for consistency */

  /* Borders and separators */
  --ifm-border-color: #ced4da; /* Neutral gray for borders */
  --ifm-toc-border-color: #e9ecef; /* Table of contents border */
  
  /* Code block styling */
  --ifm-code-background: #f8f9fa; /* Light background for code */
  --ifm-code-color: #e83e8c; /* Custom code text color */
  --docusaurus-highlighted-code-line-bg: #ffeaa7; /* Highlighted lines */
  
  /* Links */
  --ifm-link-color: #0d6efd; /* Bootstrap blue for links */
  --ifm-link-hover-color: #0b5ed7; /* Hover state */
}

/* Dark mode academic palette */
html[data-theme='dark'] {
  --ifm-color-primary: #7ca5d3; /* Lighter blue for dark mode */
  --ifm-color-primary-dark: #5a89c0; /* Darker variant */
  --ifm-color-primary-darker: #4a79b5; /* Even darker variant */
  --ifm-color-primary-darkest: #3a6095; /* Darkest variant */
  --ifm-color-primary-light: #9ec0e1; /* Lighter variant */
  --ifm-color-primary-lighter: #b7d2e8; /* Even lighter variant */
  --ifm-color-primary-lightest: #d0e4f2; /* Lightest variant */

  --ifm-background-color: #121212; /* Dark background */
  --ifm-background-surface-color: #1e1e1e; /* Surface area */
  --ifm-font-color-base: #e1e1e1; /* Light text */
  --ifm-heading-color: #d0e4f2; /* Light heading */
  --ifm-border-color: #333333; /* Darker borders */
  --ifm-code-background: #2d2d2d; /* Dark code background */
}
```

## Typography Overrides
```css
/* Academic Typography - optimized for reading */
html {
  /* Base font size for better readability */
  font-size: 16px;
}

/* Body text styling for extended reading */
.markdown,
article,
.main-wrapper {
  font-family: 'Georgia', 'Times New Roman', serif; /* Serif for body text */
  line-height: 1.6;
  font-size: 1.05rem;
  color: var(--ifm-font-color-base);
}

/* Headers maintain a clean academic hierarchy */
h1, h2, h3, h4, h5, h6 {
  font-family: 'Inter', 'Helvetica Neue', Arial, sans-serif; /* Sans-serif for headers */
  color: var(--ifm-heading-color);
  font-weight: 600;
}

/* Code typography */
code {
  font-family: 'SFMono-Regular', 'Consolas', 'Liberation Mono', 'Menlo', monospace;
  font-size: 0.9em;
  background-color: var(--ifm-code-background);
  color: var(--ifm-code-color);
  padding: 2px 6px;
  border-radius: 4px;
}

/* Blockquote styling for academic citations */
blockquote {
  border-left: 4px solid var(--ifm-color-primary);
  background-color: rgba(26, 54, 93, 0.05); /* Light blue tint */
  padding: 16px 24px;
  margin: 24px 0;
  font-style: italic;
  color: var(--ifm-font-color-base);
}

/* Table styling for academic content */
table {
  border-radius: 8px;
  overflow: hidden;
  box-shadow: 0 2px 8px rgba(0,0,0,0.08);
}

th, td {
  padding: 12px 16px;
  border-bottom: 1px solid var(--ifm-border-color);
}
```

## Component-Specific Overrides

### Navigation Bar
```css
/* Book-themed navbar */
.navbar {
  background-color: var(--ifm-background-color);
  box-shadow: 0 2px 4px rgba(0,0,0,0.05);
  border-bottom: 1px solid var(--ifm-border-color);
  padding-top: 0.5rem;
  padding-bottom: 0.5rem;
}

.navbar__logo img {
  max-height: 2.5rem;
  filter: none; /* Ensure logo displays as-is without color filters */
}

.navbar__title {
  color: var(--ifm-heading-color);
  font-weight: 600;
  margin-left: 0.5rem;
}
```

### Footer
```css
/* Academic footer */
.footer {
  background-color: var(--ifm-background-surface-color);
  border-top: 1px solid var(--ifm-border-color);
  padding-top: 2rem;
  padding-bottom: 2rem;
  color: var(--ifm-font-color-base);
}

.footer__copyright {
  color: var(--ifm-font-color-base);
  font-size: 0.9rem;
}
```

### Homepage Hero Section
```css
/* Book-themed hero section */
.hero {
  background: linear-gradient(135deg, #1a365d 0%, #2d5aa0 100%);
  color: white;
  padding-top: 4rem;
  padding-bottom: 4rem;
}

.hero__title {
  font-size: 3rem;
  font-weight: 700;
  margin-bottom: 1rem;
}

.hero__subtitle {
  font-size: 1.5rem;
  opacity: 0.9;
  max-width: 800px;
}

/* Remove any Docusaurus-specific styling */
.hero--primary {
  --ifm-hero-background-color: #1a365d;
  --ifm-hero-text-color: white;
}
```

### Content Layout
```css
/* Academic content layout */
.container.padding-top--md {
  padding-top: 2rem;
}

.container.padding-bottom--md {
  padding-bottom: 2rem;
}

/* Sidebar styling for academic navigation */
.menu {
  font-size: 0.9rem;
}

.menu__link {
  color: var(--ifm-font-color-base);
}

.menu__link--active {
  color: var(--ifm-color-primary);
  background-color: transparent;
  border-left: 3px solid var(--ifm-color-primary);
}

/* Pagination for academic reading flow */
.pagination-nav__label {
  color: var(--ifm-color-primary);
}

/* Card components for academic content */
.card {
  border: 1px solid var(--ifm-border-color);
  border-radius: 8px;
  box-shadow: 0 4px 6px rgba(0,0,0,0.05);
}
```

## Asset Rendering Validation
```css
/* Ensure proper logo rendering */
.navbar__logo img,
.theme-admonition img {
  max-width: 100%;
  height: auto;
}

/* Favicon validation - CSS cannot set favicon, but we ensure no conflicts */
/* Favicon is set via themeConfig, not CSS */

/* Responsive design for academic reading */
@media (max-width: 996px) {
  .navbar__items {
    font-size: 0.9rem;
  }
  
  .main-wrapper {
    padding-left: 0;
    padding-right: 0;
  }
  
  .container {
    padding-left: 1rem;
    padding-right: 1rem;
  }
}
```

## Validation Rules
- All CSS values must use ASCII-safe characters
- Color values must be in hex, rgba, or named CSS color formats
- Font families must be in standard CSS format with fallbacks
- All overrides must maintain accessibility standards (contrast ratios)
- Responsive design must be preserved across all modifications