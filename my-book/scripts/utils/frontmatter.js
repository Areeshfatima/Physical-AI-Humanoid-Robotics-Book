const fs = require('fs');

/**
 * Utility functions for detecting and extracting front matter
 */

/**
 * Check if a file starts with YAML front matter
 * @param {string} filePath - Path to the markdown file
 * @returns {boolean} - True if the file starts with front matter, false otherwise
 */
function hasFrontMatter(filePath) {
  const content = fs.readFileSync(filePath, 'utf8');
  return content.startsWith('---');
}

/**
 * Extract the front matter and content from a markdown file
 * @param {string} filePath - Path to the markdown file
 * @returns {object} - Object with frontMatter (string), content (string), and hasFrontMatter (boolean)
 */
function extractFrontMatter(filePath) {
  const content = fs.readFileSync(filePath, 'utf8');
  
  if (!content.startsWith('---')) {
    // No front matter, return the entire content as content
    return {
      frontMatter: null,
      content: content,
      hasFrontMatter: false
    };
  }
  
  // Find the end of the front matter (second occurrence of ---)
  const lines = content.split('\n');
  let frontMatterEndIndex = -1;
  
  // Look for the closing --- of the front matter block
  for (let i = 1; i < lines.length; i++) { // Start from 1 since we know first line is ---
    if (lines[i].trim() === '---') {
      frontMatterEndIndex = i;
      break;
    }
  }
  
  if (frontMatterEndIndex === -1) {
    // Unclosed front matter block - treat as content
    return {
      frontMatter: null,
      content: content,
      hasFrontMatter: false
    };
  }
  
  // Extract front matter and content
  const frontMatter = lines.slice(1, frontMatterEndIndex).join('\n');
  const markdownContent = lines.slice(frontMatterEndIndex + 1).join('\n');
  
  return {
    frontMatter: frontMatter,
    content: markdownContent,
    hasFrontMatter: true
  };
}

/**
 * Check if the front matter is valid YAML
 * @param {string} frontMatter - Front matter string to validate
 * @returns {boolean} - True if valid YAML, false otherwise
 */
function isValidFrontMatterYAML(frontMatter) {
  // Import yaml-parser here to avoid circular dependencies
  const { isValidYaml } = require('./yaml-parser');
  return isValidYaml(frontMatter);
}

module.exports = {
  hasFrontMatter,
  extractFrontMatter,
  isValidFrontMatterYAML
};