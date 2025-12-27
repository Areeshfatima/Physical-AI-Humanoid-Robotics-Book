# Asset Validation Rules

## Overview
This document outlines the validation requirements for all branding assets (logos, favicons, etc.) to ensure proper rendering and compatibility with the book-themed Docusaurus site.

## Logo Validation

### File Format Requirements
- **Supported formats**: PNG, JPG, SVG, WebP
- **Maximum file size**: 200KB for raster images, 50KB for SVG
- **Minimum resolution**: 200x200 pixels for standard displays
- **Recommended resolution**: 400x400 pixels for high-DPI displays

### Path Validation
- **Path format**: Must be relative to the static assets directory (`/static/img/` or similar)
- **ASCII compliance**: All path characters must be ASCII-safe (no special characters, spaces, or Unicode)
- **Structure**: Paths should follow the pattern `/img/logos/book-logo.{extension}`
- **Validation check**: Path must exist and be accessible when site is built

### Rendering Validation
- **Dimensions**: Logo should render appropriately in navbar (max height: 40px)
- **Fallback**: No text fallback should appear when image is present and valid
- **Responsive**: Logo should scale appropriately on mobile devices
- **Alternative text**: `alt` attribute must describe the book appropriately for accessibility

## Favicon Validation

### File Format Requirements
- **Supported formats**: ICO, PNG
- **Required sizes**: 16x16, 32x32, 48x48 for ICO; 192x192, 512x512 for PNG
- **Maximum file size**: 50KB for ICO, 30KB for PNG

### Path Validation
- **Path format**: Relative to the static directory or as data URI
- **ASCII compliance**: Path must contain only ASCII-safe characters
- **Multiple formats**: Ensure all required favicon formats are specified in themeConfig

## Image Asset Validation

### General Requirements
- **Format**: PNG, JPG, SVG, WebP preferred
- **Optimization**: Images should be compressed without significant quality loss
- **Alt text**: All images must have appropriate alt text for accessibility
- **Copyright**: All images must either be original or properly licensed for educational use

### Specific Validation Checks
1. **Path existence**: Verify that all image paths referenced in the theme exist
2. **Loading**: Confirm images load correctly across different browsers and devices
3. **Performance**: Ensure images don't significantly impact page load time
4. **Accessibility**: Validate that all images have appropriate alt text

## Validation Procedures

### Automated Checks
1. **Path validation**: Script that verifies all asset paths exist in the build directory
2. **Format validation**: Check that all assets are in supported formats
3. **Size validation**: Ensure assets meet size requirements
4. **Accessibility validation**: Verify alt text and other accessibility attributes

### Manual Checks
1. **Visual inspection**: Look at the site in different browsers and devices
2. **Responsive testing**: Ensure assets scale appropriately on different screen sizes
3. **Performance testing**: Verify that assets don't negatively impact load times
4. **Accessibility testing**: Use tools like axe-core to validate accessibility

## Error Handling

### Missing Assets
- **Detection**: Log all 404 errors for asset paths
- **Fallbacks**: Implement appropriate fallbacks for missing assets
- **Reporting**: Generate reports on missing or invalid assets

### Invalid Assets
- **Format issues**: Detect and report unsupported file formats
- **Size issues**: Flag assets that exceed size limitations
- **Corruption**: Identify corrupted image files that won't render

## Implementation Requirements

1. **Validation script**: Create a validation script that runs during build process
2. **Automated checking**: Ensure all asset checks are performed automatically
3. **Reporting**: Generate clear reports of validation results
4. **Prevention**: Prevent builds with invalid assets from being deployed

## Success Criteria

- All assets render correctly across browsers and devices
- No 404 errors for asset files
- Average page load time remains under 3 seconds
- All assets pass accessibility validation
- All asset paths use only ASCII-safe characters
- Build process validates assets deterministically