const fs = require('fs');
const { extractFrontMatter } = require('./frontmatter');

/**
 * Utility function for extracting content from markdown files
 */

/**
 * Extract only the content part (without front matter) from a markdown file
 * @param {string} filePath - Path to the markdown file
 * @returns {string} - The content without front matter
 */
function extractContentOnly(filePath) {
  const { content } = extractFrontMatter(filePath);
  return content;
}

/**
 * Get the full content of a file but verify content hasn't been altered
 * @param {string} filePath - Path to the markdown file
 * @param {string} originalContent - Original content to compare against
 * @returns {object} - Object with content and verification status
 */
function extractAndVerifyContent(filePath, originalContent = null) {
  const { content } = extractFrontMatter(filePath);
  
  const contentUnchanged = !originalContent || content === originalContent;
  
  return {
    content,
    contentUnchanged
  };
}

module.exports = {
  extractContentOnly,
  extractAndVerifyContent
};