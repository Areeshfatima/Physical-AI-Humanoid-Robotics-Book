const glob = require('glob');
const fs = require('fs').promises;
const path = require('path');

/**
 * Scans the documentation directory for all .md and .mdx files
 * @param {string} docsDir - Directory to scan (default: 'my-book/docs/')
 * @returns {Promise<string[]>} List of markdown file paths
 */
async function scanDocumentationFiles(docsDir = 'my-book/docs/') {
  // Get all .md and .mdx files in the docs directory and subdirectories
  const mdFiles = glob.sync(`${docsDir}/**/*.md`, { absolute: true });
  const mdxFiles = glob.sync(`${docsDir}/**/*.mdx`, { absolute: true });
  
  // Combine and return the file paths
  return [...mdFiles, ...mdxFiles];
}

/**
 * Checks if a file exists
 * @param {string} filePath - Path to the file
 * @returns {Promise<boolean>} True if file exists
 */
async function fileExists(filePath) {
  try {
    await fs.access(filePath);
    return true;
  } catch {
    return false;
  }
}

module.exports = {
  scanDocumentationFiles,
  fileExists
};