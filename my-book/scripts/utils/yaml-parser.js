const yaml = require('js-yaml');
const fs = require('fs');

/**
 * Utility functions for parsing and validating YAML front matter
 */

/**
 * Parse YAML from a string
 * @param {string} yamlString - The YAML string to parse
 * @returns {object|null} - The parsed YAML object or null if parsing fails
 */
function parseYaml(yamlString) {
  try {
    return yaml.load(yamlString);
  } catch (error) {
    console.error('YAML parsing error:', error.message);
    return null;
  }
}

/**
 * Check if a string is valid YAML
 * @param {string} yamlString - The YAML string to validate
 * @returns {boolean} - True if valid YAML, false otherwise
 */
function isValidYaml(yamlString) {
  try {
    yaml.load(yamlString);
    return true;
  } catch (error) {
    return false;
  }
}

/**
 * Ensure a string value is properly quoted for YAML
 * @param {string} value - The value to quote
 * @returns {string} - The properly quoted value
 */
function quoteYamlValue(value) {
  if (typeof value !== 'string') {
    return value;
  }
  
  // If the value is already quoted, return as is
  if ((value.startsWith('"') && value.endsWith('"')) || 
      (value.startsWith("'") && value.endsWith("'"))) {
    return value;
  }
  
  // Check if the value contains characters that require quoting
  if (/[<>"'&@`]/.test(value) || 
      value.includes(': ') || 
      value.includes('{') || 
      value.includes('}') || 
      value.includes('[') || 
      value.includes(']') ||
      value.includes('#') ||
      value.trim() !== value) { // contains leading/trailing spaces
    // Use double quotes for values that contain special characters
    return `"${value.replace(/"/g, '\\"')}"`;
  }
  
  // For simple values, no need to quote
  return value;
}

module.exports = {
  parseYaml,
  isValidYaml,
  quoteYamlValue
};