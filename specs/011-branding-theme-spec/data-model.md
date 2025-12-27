# Data Model: Docusaurus Book Branding and Theming

## Overview
This document describes the data model for the branding and theming implementation. Since this feature is primarily about UI/visual changes, the data model focuses on configuration and asset structures.

## Branding Configuration

### ThemeConfig Structure
```javascript
{
  // Navbar configuration
  navbar: {
    title: "Book Title",           // Book title shown in navbar
    logo: {
      alt: "Book Logo",            // Alt text for accessibility
      src: "/img/book-logo.svg",   // Path to logo image
      srcDark: "/img/book-logo-dark.svg" // Path to dark mode logo (optional)
    },
    items: [/* navigation items */]
  },

  // Footer configuration
  footer: {
    copyright: "Copyright © 2025 Book Title. All rights reserved.",
    links: [/* footer link items */],
    logo: {
      alt: "Book Logo",
      src: "/img/book-logo.svg",
      href: "https://website-url.com"
    }
  },

  // Metadata configuration
  metadata: [
    {
      name: "description",
      content: "Academic textbook on Physical AI & Humanoid Robotics"
    },
    {
      name: "keywords",
      content: "ai, robotics, textbook, education"
    }
  ],

  // Color mode configuration
  colorMode: {
    defaultMode: "light",
    disableSwitch: false,
    respectPrefersColorScheme: true
  },

  // Favicon configuration
  favicon: "/img/favicon.ico"
}
```

## Asset Structure

### Logo Assets
- `static/img/book-logo.svg` - Main logo for light theme
- `static/img/book-logo-dark.svg` - Alternative logo for dark theme
- `static/img/book-logo-192.png` - Larger logo for social media sharing and manifest
- `static/img/favicon.ico` - Favicon file
- `static/img/favicon-16x16.png`, `static/img/favicon-32x32.png` - Alternative favicon sizes

### Branding Assets
- `src/css/custom.css` - Custom CSS overrides
- `src/css/academic-theme.css` - Academic-themed styles
- `src/css/rtl-support.css` - Right-to-left text support

## Validation Rules

### Configuration Validation
- All asset paths must use forward slashes (ASCII compliance)
- All asset paths must be relative to the static directory
- All configuration values must be ASCII characters only
- All branding elements must have appropriate fallbacks

### Asset Validation
- Logo dimensions: minimum 200x200 pixels, recommended 400x400
- Favicon formats: ICO, PNG with required sizes
- File sizes: logos under 200KB, favicons under 50KB
- Accessibility: all images must have appropriate alt text

## State Transitions

### Theme State
- Light mode (default)
- Dark mode (user preference or toggle)
- System preference (follows OS settings)

### Language State
- Primary language (default)
- Secondary language (user selected)
- RTL language state (for right-to-left text display)