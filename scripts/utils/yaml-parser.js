const yaml = require('js-yaml');
const fs = require('fs').promises;

/**
 * Safely parse YAML content
 * @param {string} yamlContent - YAML string to parse
 * @returns {Object|null} Parsed object or null if invalid
 */
function parseYaml(yamlContent) {
  try {
    return yaml.load(yamlContent);
  } catch (error) {
    console.error('YAML parsing error:', error.message);
    return null;
  }
}

/**
 * Safely dump object to YAML content
 * @param {Object} obj - Object to convert to YAML
 * @returns {string} YAML string
 */
function dumpYaml(obj) {
  try {
    // Use best practices for dumping to ensure special characters are handled correctly
    return yaml.dump(obj, {
      quotingType: '"', // Use double quotes by default to handle special characters
      noRefs: true,
      noCompatMode: true
    });
  } catch (error) {
    console.error('YAML dumping error:', error.message);
    return '';
  }
}

/**
 * Validates if a string is valid YAML
 * @param {string} yamlContent - YAML string to validate
 * @returns {boolean} True if valid YAML
 */
function isValidYaml(yamlContent) {
  try {
    yaml.load(yamlContent);
    return true;
  } catch {
    return false;
  }
}

module.exports = {
  parseYaml,
  dumpYaml,
  isValidYaml
};