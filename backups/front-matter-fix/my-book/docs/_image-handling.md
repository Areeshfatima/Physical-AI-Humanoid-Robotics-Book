---
id: image-handling
title: Image Handling
---





# Image Handling Documentation

This document describes the image handling system implemented for the Physical AI & Humanoid Robotics textbook.

## Overview

The image handling system provides:

1. A structured approach to organizing images in the textbook
2. Validation of images for size, format, and security
3. Placeholder system for draft content
4. Accessibility compliance for all images
5. AI prompt generation for future image creation

## Directory Structure

Images are organized in the following structure:

```
static/
└── img/
    ├── 001-docusaurus-textbook/
    │   ├── chapter-1/
    │   ├── chapter-2/
    │   ├── chapter-3/
    │   └── chapter-4/
    ├── 003-digital-twin-gazebo-unity/
    │   ├── chapter-1/
    │   ├── chapter-2/
    │   ├── chapter-3/
    │   └── chapter-4/
    ├── 004-isaac-sim-vslam-nav/
    │   ├── chapter-1/
    │   ├── chapter-2/
    │   ├── chapter-3/
    │   └── chapter-4/
    └── 005-vla-systems/
        ├── chapter-1/
        ├── chapter-2/
        ├── chapter-3/
        └── chapter-4/
```

Each module has its own directory with subdirectories for each chapter.

## Image Requirements

- Supported formats: PNG, JPG, GIF, SVG
- Maximum file size: 5MB
- All images must have descriptive alt text for accessibility
- SVG images are sanitized to prevent security vulnerabilities

## Placeholder System

During the drafting phase, placeholder images are used when final images are not available. The system includes:

- Generic placeholder images in each chapter directory
- Module-specific placeholder images
- A main placeholder image in the root img directory

## Validation Tools

The system includes several validation tools:

1. `scripts/scan-images.js` - Scans markdown files for broken image references
2. `scripts/validate-images.js` - Validates image size, format, and security
3. `scripts/validate-alt-text.js` - Ensures all images have appropriate alt text
4. `scripts/generate-image-prompts.js` - Creates AI prompts for future image generation

## Adding Images to Content

To add an image to your markdown content:

```markdown
![Descriptive alt text](/img/module-directory/chapter-directory/image-name.format)
```

For example:
```markdown
![Architecture diagram showing system components](/img/001-docusaurus-textbook/chapter-1/architecture_diagram.svg)
```

## Image Generation Process

For each chapter, the system generates AI prompts that can be used to create appropriate images. These prompts are stored in `scripts/resources/_image-prompts.md` and include:

- Context from the chapter content
- Technical concepts that could benefit from visualization
- Specific AI prompts tailored to the content

## Accessibility Compliance

All images must meet accessibility standards:

- Alt text must be descriptive and meaningful
- Color contrast must meet WCAG 2.1 AA standards
- Complex images should include extended descriptions
- Images should be understandable in grayscale

## Security Considerations

SVG images are sanitized to remove potentially dangerous content such as:

- JavaScript code
- Event handlers (onload, onclick, etc.)
- Malicious URLs

The sanitization process automatically removes these elements when saving the SVG file.