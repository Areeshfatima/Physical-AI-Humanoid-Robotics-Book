const fs = require('fs');
const { extractFrontMatter } = require('./frontmatter');

/**
 * Utility functions for validating front matter
 */

/**
 * Check if a file has missing required front matter fields
 * @param {string} filePath - Path to the markdown file
 * @returns {object} - Object with 'missingFields' array and 'hasRequiredFields' boolean
 */
function hasMissingRequiredFields(filePath) {
  const { frontMatter, hasFrontMatter } = extractFrontMatter(filePath);
  
  if (!hasFrontMatter || !frontMatter) {
    return {
      hasRequiredFields: false,
      missingFields: ['id', 'title'],
      hasFrontMatter: false
    };
  }
  
  // Parse the front matter to check for required fields
  const { parseYaml } = require('./yaml-parser');
  const parsedFrontMatter = parseYaml(frontMatter);
  
  if (!parsedFrontMatter) {
    // If front matter can't be parsed, it's considered invalid
    return {
      hasRequiredFields: false,
      missingFields: ['id', 'title'],
      hasFrontMatter: true
    };
  }
  
  const missingFields = [];
  if (!parsedFrontMatter.id) {
    missingFields.push('id');
  }
  if (!parsedFrontMatter.title) {
    missingFields.push('title');
  }
  
  return {
    hasRequiredFields: missingFields.length === 0,
    missingFields,
    hasFrontMatter: true
  };
}

/**
 * Check if front matter has valid YAML syntax
 * @param {string} filePath - Path to the markdown file
 * @returns {boolean} - True if front matter has valid YAML, false otherwise
 */
function hasValidFrontMatterYAML(filePath) {
  const { frontMatter, hasFrontMatter } = extractFrontMatter(filePath);
  
  if (!hasFrontMatter || !frontMatter) {
    return false;
  }
  
  const { isValidYaml } = require('./yaml-parser');
  return isValidYaml(frontMatter);
}

module.exports = {
  hasMissingRequiredFields,
  hasValidFrontMatterYAML
};