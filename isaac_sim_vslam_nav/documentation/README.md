# Isaac Sim VSLAM Navigation Documentation

This directory contains Docusaurus documentation for the Isaac Sim VSLAM Navigation module for humanoid robotics.

## Structure

- `docs/` - Contains the main documentation files in Markdown format
- `src/` - Contains source files including CSS customizations
- `docusaurus.config.js` - Main Docusaurus configuration file
- `sidebars.js` - Sidebar navigation configuration
- `package.json` - Node.js package configuration for Docusaurus

## Local Development

1. Install dependencies:
```bash
npm install
```

2. Start the development server:
```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

To build the documentation site:

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

The site is set up to be deployed to GitHub Pages. See the `docusaurus.config.js` file for deployment settings.

## Documentation Content

The documentation covers:

1. Isaac Sim integration for humanoid robotics
2. Isaac ROS VSLAM implementation
3. Nav2 path planning for bipedal locomotion

Each section includes setup instructions, configuration guides, and performance validation information.