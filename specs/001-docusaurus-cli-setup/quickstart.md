# Quickstart: Docusaurus CLI Setup

## Prerequisites

- Node.js 18+ installed
- npm package manager
- Git (for cloning the repository)

## Setup Steps

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd physical-ai-humanoid-robotics
   ```

2. **Navigate to the textbook project**
   ```bash
   cd my-book
   ```

3. **Install dependencies**
   ```bash
   npm install
   ```
   
   This will install all required Docusaurus packages and dependencies.

4. **Verify Docusaurus CLI is available**
   ```bash
   npx docusaurus --version
   ```

5. **Start the development server**
   ```bash
   npm start
   ```
   
   This should start the Docusaurus development server on http://localhost:3000

## Troubleshooting

If you encounter the "docusaurus: not found" error:

1. Ensure Docusaurus packages are properly installed:
   ```bash
   npm install @docusaurus/core @docusaurus/preset-classic
   ```

2. Check that the start script exists in package.json:
   ```json
   {
     "scripts": {
       "start": "docusaurus start"
     }
   }
   ```

3. Try clearing npm cache and reinstalling:
   ```bash
   npm cache clean --force
   rm -rf node_modules package-lock.json
   npm install
   ```

## Validating the Setup

Follow the testing strategy to ensure the setup is working properly:

1. **npm install**: Run this command to install all dependencies
   ```bash
   npm install
   ```
   Verify that all dependencies install without errors

2. **npm start**: Run this command to launch the development server
   ```bash
   npm start
   ```
   Verify that the development server starts without errors

3. **Localhost verification**: Access the documentation at http://localhost:3000
   Verify that:
   - The website is accessible at http://localhost:3000
   - All documentation pages load without errors
   - Hot reloading works when content is modified
   - The server starts within 30 seconds (as specified in the requirements)