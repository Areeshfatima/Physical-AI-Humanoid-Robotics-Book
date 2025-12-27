# Quickstart Guide: Docusaurus Book Branding Implementation

## Overview
This guide provides step-by-step instructions to implement the branding changes for the Docusaurus-based technical textbook.

## Prerequisites
- Node.js 18+ installed
- npm or yarn package manager
- Access to the project repository
- Book logo and favicon assets prepared

## Setup Steps

### 1. Prepare Assets
1. Place the book logo in `static/img/book-logo.svg`
2. Place the dark mode logo in `static/img/book-logo-dark.svg` (optional)
3. Place the favicon in `static/img/favicon.ico`
4. Add additional favicon sizes if needed:
   - `static/img/favicon-16x16.png`
   - `static/img/favicon-32x32.png`

### 2. Update Theme Configuration
1. Open `docusaurus.config.js`
2. Update the `themeConfig` section with your book's branding:
   ```javascript
   themeConfig: {
     navbar: {
       title: "Your Book Title",
       logo: {
         alt: "Book Logo",
         src: "/img/book-logo.svg",
         srcDark: "/img/book-logo-dark.svg", // Optional
       },
       items: [
         // Your navigation items
       ],
     },
     footer: {
       copyright: `Copyright © ${new Date().getFullYear()} Your Book Title. All rights reserved.`,
       logo: {
         alt: "Book Logo",
         src: "/img/book-logo.svg",
       },
       links: [
         // Your footer links
       ],
     },
     // Other theme config options...
   },
   ```

### 3. Add Custom CSS
1. Create or update `src/css/custom.css` with academic styling
2. Add the CSS rules from the `css-overrides.md` file
3. Ensure accessibility standards are met

### 4. Implement Multi-language Support
1. Create the `i18n/` directory if it doesn't exist
2. Follow Docusaurus i18n documentation to set up language files
3. Add RTL support CSS if needed

### 5. Validate Implementation
1. Run `npm run start` to start the development server
2. Verify all branding elements display correctly:
   - Navbar shows book title and logo
   - Footer shows book copyright information
   - Favicon displays correctly
   - Logo renders without text fallback
   - Academic color palette applied
3. Test on multiple browsers and devices
4. Verify page load times are under 3 seconds

## Common Issues and Solutions

### Logo Not Displaying
- Verify the path in themeConfig matches the actual file location
- Check that the file format is supported (PNG, JPG, SVG, WebP)
- Ensure the file isn't corrupted

### Text Fallback Still Showing
- Make sure the logo configuration in themeConfig is properly formatted
- Check that the alt text is appropriate

### Performance Issues
- Optimize image assets for web delivery
- Use appropriate file formats and compression

## Verification Checklist
- [ ] All Docusaurus branding removed/replaced with book branding
- [ ] Logo renders correctly without text fallback
- [ ] Academic color palette applied consistently
- [ ] Navbar shows only book title
- [ ] Footer shows only book copyright
- [ ] Appropriate fonts for textbook reading implemented
- [ ] Favicon replaced with book-specific version
- [ ] Homepage hero text is book-oriented
- [ ] Multi-language support implemented (if needed)
- [ ] Page load times under 3 seconds
- [ ] All assets use ASCII-safe paths
- [ ] Build output is deterministic