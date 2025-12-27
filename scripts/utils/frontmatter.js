const fs = require('fs').promises;
const yaml = require('js-yaml');

/**
 * Checks if a file starts with valid YAML front matter (--- on line 1)
 * @param {string} filePath - Path to the file to check
 * @returns {Promise<boolean>} True if file starts with valid YAML front matter
 */
async function hasValidFrontMatterAtStart(filePath) {
  try {
    const content = await fs.readFile(filePath, 'utf8');
    const lines = content.split('\n');
    
    // Check if the first line is exactly '---'
    return lines.length > 0 && lines[0].trim() === '---';
  } catch (error) {
    console.error(`Error checking front matter for ${filePath}:`, error);
    return false;
  }
}

/**
 * Checks if a file has missing YAML front matter
 * @param {string} filePath - Path to the file to check
 * @returns {Promise<boolean>} True if file is missing YAML front matter
 */
async function hasMissingFrontMatter(filePath) {
  try {
    const hasFrontMatter = await hasValidFrontMatterAtStart(filePath);
    return !hasFrontMatter;
  } catch (error) {
    console.error(`Error checking missing front matter for ${filePath}:`, error);
    return true; // Consider as missing in case of error
  }
}

/**
 * Checks if a file has invalid YAML syntax in existing front matter
 * @param {string} filePath - Path to the file to check
 * @returns {Promise<boolean>} True if file has invalid YAML syntax in front matter
 */
async function hasInvalidYamlSyntax(filePath) {
  try {
    const content = await fs.readFile(filePath, 'utf8');
    
    if (!content.startsWith('---\n')) {
      return false; // No front matter to validate
    }
    
    // Extract the front matter (content between the first --- and the next ---)
    const lines = content.split('\n');
    if (lines.length < 2) return false;
    
    // Find the end of the front matter
    let frontMatterEndIndex = -1;
    for (let i = 1; i < lines.length; i++) {
      if (lines[i].trim() === '---') {
        frontMatterEndIndex = i;
        break;
      }
    }
    
    if (frontMatterEndIndex === -1) {
      // No closing --- found, so the front matter is incomplete/invalid
      return true;
    }
    
    // Extract the YAML content
    const yamlContent = lines.slice(1, frontMatterEndIndex).join('\n');
    
    try {
      // Try to parse the YAML
      yaml.load(yamlContent);
      // If parsing succeeds, the YAML is valid
      return false;
    } catch (yamlError) {
      // If parsing fails, the YAML is invalid
      return true;
    }
  } catch (error) {
    console.error(`Error checking YAML syntax for ${filePath}:`, error);
    return true; // Consider as invalid in case of error
  }
}

/**
 * Parses YAML front matter from a file
 * @param {string} filePath - Path to the file to parse
 * @returns {Promise<Object|null>} Parsed YAML object or null if no valid front matter
 */
async function parseYamlFrontMatter(filePath) {
  try {
    const content = await fs.readFile(filePath, 'utf8');
    
    if (!content.startsWith('---\n')) {
      return null; // No front matter to parse
    }
    
    // Extract the front matter (content between the first --- and the next ---)
    const lines = content.split('\n');
    
    // Find the end of the front matter
    let frontMatterEndIndex = -1;
    for (let i = 1; i < lines.length; i++) {
      if (lines[i].trim() === '---') {
        frontMatterEndIndex = i;
        break;
      }
    }
    
    if (frontMatterEndIndex === -1) {
      return null; // No closing --- found
    }
    
    // Extract the YAML content
    const yamlContent = lines.slice(1, frontMatterEndIndex).join('\n');
    
    try {
      // Parse the YAML
      const parsedYaml = yaml.load(yamlContent);
      return parsedYaml;
    } catch (yamlError) {
      console.error(`Error parsing YAML for ${filePath}:`, yamlError);
      return null;
    }
  } catch (error) {
    console.error(`Error parsing front matter for ${filePath}:`, error);
    return null;
  }
}

module.exports = {
  hasValidFrontMatterAtStart,
  hasMissingFrontMatter,
  hasInvalidYamlSyntax,
  parseYamlFrontMatter
};