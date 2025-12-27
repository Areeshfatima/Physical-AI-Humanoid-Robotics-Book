#!/usr/bin/env node

/**
 * Script to scan all module markdown files for missing or broken image references
 * This script will scan markdown files in the /docs directory and check if referenced images exist in /static/img/
 */

const fs = require('fs');
const path = require('path');

// Define the project paths
const DOCS_DIR = path.join(__dirname, '..', 'docs');
const STATIC_IMG_DIR = path.join(__dirname, '..', 'static', 'img');

// Regex to match markdown image references
// Matches both ![alt text](path) and <img src="path"> patterns
const IMG_REGEX = /!\[.*?\]\(([^)]+)\)|<img[^>]*src=["']([^"']+)["']/g;

// Function to find all markdown files in docs directory recursively
function findMarkdownFiles(dir = DOCS_DIR) {
  let results = [];
  const items = fs.readdirSync(dir);

  for (const item of items) {
    const itemPath = path.join(dir, item);
    const stat = fs.statSync(itemPath);

    if (stat.isDirectory()) {
      results = results.concat(findMarkdownFiles(itemPath));
    } else if (path.extname(item) === '.md') {
      results.push(itemPath);
    }
  }

  return results;
}

// Function to check if an image path exists
function imageExists(imagePath) {
  if (path.isAbsolute(imagePath)) {
    // If it's an absolute path starting with /static/img/, check in the static/img directory
    if (imagePath.startsWith('/img/')) {
      const relativePath = imagePath.substring(1); // Remove leading slash
      const fullPath = path.join(STATIC_IMG_DIR, relativePath);
      return fs.existsSync(fullPath);
    }
  } else {
    // For relative paths in markdown files, resolve relative to the markdown file's location
    // For this scan, we focus on absolute paths used by Docusaurus
    return false;
  }
  return false;
}

// Main function to scan for broken image references
function scanForBrokenImages() {
  const markdownFiles = findMarkdownFiles();
  const brokenReferences = [];
  const allReferences = [];

  console.log(`Scanning ${markdownFiles.length} markdown files for broken image references...\n`);

  markdownFiles.forEach(file => {
    const content = fs.readFileSync(file, 'utf8');
    let match;
    
    while ((match = IMG_REGEX.exec(content)) !== null) {
      // Extract the image path from either markdown or html format
      const imagePath = match[1] || match[2]; // First group for markdown, second for html
      
      if (!imagePath) continue;
      
      allReferences.push({
        file: path.relative(DOCS_DIR, file),
        imagePath: imagePath,
      });
      
      if (!imageExists(imagePath)) {
        brokenReferences.push({
          file: path.relative(DOCS_DIR, file),
          imagePath: imagePath,
        });
      }
    }
  });

  console.log(`Found ${allReferences.length} total image references`);
  console.log(`Found ${brokenReferences.length} broken image references\n`);

  if (brokenReferences.length > 0) {
    console.log('Broken image references found:');
    brokenReferences.forEach(ref => {
      console.log(`  - In file: ${ref.file}`);
      console.log(`    Image reference: ${ref.imagePath}`);
      console.log('');
    });
  } else {
    console.log('No broken image references found! All images are properly linked.');
  }

  return {
    totalReferences: allReferences.length,
    brokenReferences: brokenReferences.length,
    brokenList: brokenReferences
  };
}

// Run the scan
if (require.main === module) {
  const results = scanForBrokenImages();
  
  // Exit with error code if broken references found
  if (results.brokenReferences > 0) {
    process.exit(1);
  }
}

module.exports = scanForBrokenImages;