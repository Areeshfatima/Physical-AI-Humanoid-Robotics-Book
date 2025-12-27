const fs = require('fs').promises;
const path = require('path');
const { glob } = require('glob');
const yaml = require('js-yaml');

/**
 * Recursively scans documentation files in a directory
 * @param {string} docsDir - Directory to scan (default: 'my-book/docs/')
 * @returns {Promise<string[]>} List of markdown file paths
 */
async function scanDocumentationFiles(docsDir = 'my-book/docs/') {
  try {
    const files = await glob(docsDir + '**/*.{md,mdx}', { absolute: true, ignore: ['**/node_modules/**'] });
    // Filter out hidden files (starting with _)
    const nonHiddenFiles = files.filter(file => {
      const basename = path.basename(file);
      return !basename.startsWith('_');
    });
    return nonHiddenFiles;
  } catch (error) {
    throw error;
  }
}

/**
 * Checks if a file is a hidden file (starts with underscore)
 * @param {string} filePath - Path to the file to check
 * @returns {boolean} True if file is hidden (starts with underscore)
 */
function isHiddenFile(filePath) {
  const basename = path.basename(filePath);
  return basename.startsWith('_');
}

module.exports = {
  scanDocumentationFiles,
  isHiddenFile
};