# Quickstart Guide: Docusaurus Textbook UI Enhancement

## Overview
This guide provides step-by-step instructions to set up and run the enhanced Physical AI Humanoid Robotics textbook website with the new UI elements.

## Prerequisites
- Node.js (version 16.x or higher)
- npm or yarn package manager
- Git for version control
- A modern web browser for testing

## Setup Instructions

### 1. Clone the Repository
```bash
git clone https://github.com/Areeshfatima/Physical-AI-Humanoid-Robotics-Book.git
cd Physical-AI-Humanoid-Robotics-Book
```

### 2. Navigate to the Docusaurus Directory
```bash
cd docusaurus
```

### 3. Install Dependencies
```bash
npm install
```

### 4. Configuration Updates
The following configurations have been updated for the UI enhancements:

#### Logo Configuration
The logo has been configured in `docusaurus.config.js`:
- Path: `/img/logo.svg` 
- Alt text: "Physical AI & Humanoid Robotics Textbook"
- Responsive variants provided for different screen densities

#### Sidebar Configuration
The sidebar has been reordered in `sidebars.js`:
- Items appear in the exact order: Introduction, Module 1, Module 2, Module 3, Module 4
- No auto-generated items are present

#### GitHub Link Configuration
The GitHub link has been updated in `docusaurus.config.js`:
- Points to: https://github.com/Areeshfatima/Physical-AI-Humanoid-Robotics-Book.git

#### Theme Customization
Academic, clean, and minimal styling has been applied via `src/css/custom.css`:
- Professional color scheme suitable for educational content
- Typography optimized for readability
- Responsive design for all device sizes

### 5. Run the Development Server
```bash
npm run start
```

The website will be available at `http://localhost:3000`

### 6. Verify UI Enhancements
After starting the development server, verify these enhancements:

1. **Logo Rendering**: Check that the logo appears correctly in the navbar
2. **Hero Section**: Verify that robotics/AI-related imagery is displayed instead of generic content
3. **Sidebar Navigation**: Confirm that sidebar items appear in the correct order: Introduction, Module 1-4
4. **GitHub Link**: Ensure the GitHub link navigates to the correct repository
5. **Theme**: Validate that academic, clean, and minimal styling is applied
6. **Responsive Design**: Test the site on different screen sizes to ensure it's fully responsive

## Building for Production
To build the site for production deployment:

```bash
npm run build
```

This creates a `build` directory with the optimized static files.

## Deployment
The built site can be deployed to:
- GitHub Pages
- Vercel
- Any static hosting service

## Troubleshooting

### Logo Not Appearing
- Ensure the logo file is in the `static/img/` directory
- Check that the path in `docusaurus.config.js` is correct
- Verify file permissions allow serving the image

### Sidebar Order Incorrect
- Review `sidebars.js` to ensure the order is explicitly defined
- Clear Docusaurus cache: `npx docusaurus clear`

### Images Not Loading
- Check that all image paths are relative to the `static/` directory
- Verify file formats are supported by browsers

### Accessibility Issues
- Run automated accessibility tests with tools like axe-core
- Verify all images have appropriate alt text
- Check color contrast ratios meet WCAG 2.1 AA standards

## Next Steps
After successfully setting up the enhanced UI:
1. Review the content in the `docs/` directory to ensure it aligns with the new UI
2. Test all navigation paths to confirm they work as expected
3. Validate accessibility compliance using tools like Lighthouse
4. Test across different browsers and devices to ensure consistency