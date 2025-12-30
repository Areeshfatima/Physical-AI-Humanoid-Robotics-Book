# Image Handling Architecture Implementation Guide

This document details the implementation of the image handling architecture for the Docusaurus textbook project, focusing on Markdown references and the integration with the static asset system.

## Architecture Overview

The image handling system is designed with the following principles:
- All images are stored exclusively in `/my-book/static/img/`
- Images are organized by module and chapter
- Markdown files reference images using relative paths
- The system gracefully handles missing or invalid images
- All images meet accessibility standards

## Markdown Image Reference Patterns

### Standard Image Reference

Images in Markdown should be referenced using this pattern:
```markdown
![Alt text describing the image](/img/module-folder/chapter-folder/image-name.png)
```

### Example Implementation

In a chapter file located at `/my-book/docs/module-1-ros2/chapter-1.md`:
```markdown
# Chapter 1: Introduction

Here's an overview of the system architecture:

![System architecture diagram showing components and relationships](/img/001-docusaurus-textbook/chapter-1/architecture_diagram.svg)

The above diagram illustrates the key components of the system.
```

### Image with Custom Styling

For images requiring specific styling, use HTML within Markdown:
```html
<img 
  src="/img/001-docusaurus-textbook/chapter-1/process_flow.png" 
  alt="Process flow diagram showing the sequence of operations" 
  class="image-style-class"
  style="width: 80%; display: block; margin: auto;"
/>
```

## Implementation Requirements

### 1. File Path Standards
- All image paths must be relative to the static directory
- Paths should follow the format: `/img/<module>/<chapter>/<filename>.<extension>`
- Use consistent naming conventions with underscores and descriptive names
- File extensions must match supported formats: .png, .jpg, .gif, .svg

### 2. Alt Text Requirements
- All images must include descriptive alt text
- Alt text should describe both the content and function of the image
- For complex images, include enough detail to understand the concept without seeing the image
- For decorative images, use empty alt text: `![](path)`

### 3. Image Optimization
- Images should be optimized for web delivery
- SVG should be used for diagrams and illustrations when possible
- PNG should be used for screenshots and images requiring transparency
- JPG should be used for photographs
- File sizes should not exceed 5MB

### 4. Accessibility Standards
- Color contrast in images must meet WCAG 2.1 AA standards
- Complex diagrams should include extended descriptions either in alt text or surrounding content
- Images should be understandable when viewed in grayscale
- All informative images must have alt text; decorative images may have empty alt text

## Build Process Integration

### 1. Image Validation
During the build process, the following checks should be performed:
- Verify all image paths in Markdown files resolve to existing files
- Check that all images are within size limits
- Validate that all required alt text is present
- Ensure images are in supported formats

### 2. Error Handling
If an image reference is invalid, the build process should:
- Display a generic placeholder image
- Log a warning for the missing image
- Continue building without breaking the page layout
- Generate a report of broken image references for remediation

### 3. Performance Optimization
The build process should:
- Optimize images for web delivery
- Implement lazy loading for images below the fold
- Generate appropriate image sizes for responsive design
- Cache images appropriately

## Context7 MCP Server Integration

The Context7 MCP server provides structure awareness for the image handling system:
- Understanding the folder hierarchy for images
- Validating image references against available files
- Providing intelligent suggestions for image placement
- Checking for consistency in image usage across modules

## Qwen CLI Integration

The Qwen CLI executes Spec-Kit Plus steps to ensure:
- Consistent image folder structure
- Proper validation of image references
- Automated checks for accessibility compliance
- Quality assurance for image loading performance

## Example Markdown Implementation

Here's a complete example of how to implement images in a textbook chapter:

```markdown
---
title: "Chapter 1: System Architecture"
sidebar_position: 1
---

# System Architecture Overview

This chapter introduces the core components of the AI textbook system and how they interact.

## Core Components

The system consists of several key components that work together to provide a comprehensive learning experience.

![Architecture diagram showing the main components: Docusaurus framework, static assets, image handling system, and user interface](/img/001-docusaurus-textbook/chapter-1/main_components.svg)

### Component Details

The Docusaurus framework manages content rendering and site generation. It's responsible for:

- Processing Markdown files
- Applying themes and styles
- Generating static HTML pages
- Optimizing for search engines

![Docusaurus processing flow showing input Markdown files and output HTML pages](/img/001-docusaurus-textbook/chapter-1/docusaurus_flow.png)

## Module Structure

Each module in our textbook follows a consistent structure designed to facilitate learning:

![Diagram showing the typical structure of a module with its chapters and associated images](/img/001-docusaurus-textbook/chapter-1/module_structure.svg)

This structure ensures that learners have:
1. Clear learning objectives
2. Relevant visual materials 
3. Practical examples
4. Assessment opportunities

## Image Guidelines

When adding images to your textbook content, please follow these guidelines:

1. All images must be placed in the appropriate `/static/img/` subdirectory
2. Use descriptive filenames that indicate the content and purpose
3. Always include alt text that describes the image's content and function
4. Ensure images are optimized for web delivery (under 5MB)
5. Verify that images meet accessibility standards

![Example of good vs bad image implementation showing proper alt text and file organization](/img/001-docusaurus-textbook/chapter-1/image_guidelines.png)

For more information on image handling best practices, refer to our [detailed strategy guide](./module-image-strategy.md).
```

## Validation and Testing

To ensure the image handling architecture works correctly:

1. Before committing changes:
   - Verify all image paths are correct
   - Test that images display properly in the development server
   - Confirm alt text is descriptive and appropriate

2. During the build process:
   - Check for broken image references
   - Verify image loading times
   - Ensure all accessibility requirements are met

3. After deployment:
   - Test images on different devices and browsers
   - Verify responsive behavior
   - Check that error handling works as expected