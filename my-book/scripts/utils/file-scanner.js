const fs = require('fs');
const path = require('path');
const glob = require('glob');

/**
 * Utility function to scan and find all .md and .mdx files in the docs directory
 * @param {string} docsDir - The directory to scan (default is 'my-book/docs/')
 * @returns {string[]} - Array of file paths
 */
function scanMarkdownFiles(docsDir = './docs/') {
  // Use glob to find all .md and .mdx files
  const mdFiles = glob.sync(path.join(docsDir, '**/*.md'));
  const mdxFiles = glob.sync(path.join(docsDir, '**/*.mdx'));

  // Combine and return the file paths
  return [...mdFiles, ...mdxFiles];
}

module.exports = {
  scanMarkdownFiles
};