const fs = require('fs').promises;
const yaml = require('js-yaml');

/**
 * Checks if a file has valid front matter at the start
 * @param {string} filePath - Path to the file to check
 * @returns {Promise<boolean>} True if file starts with valid front matter (---)
 */
async function hasValidFrontMatterAtStart(filePath) {
  try {
    const content = await fs.readFile(filePath, 'utf8');
    const lines = content.split('\n');
    return lines.length > 0 && lines[0].trim() === '---';
  } catch (error) {
    console.error(`Error checking front matter for ${filePath}:`, error);
    return false;
  }
}

/**
 * Checks if a file is missing front matter
 * @param {string} filePath - Path to the file to check
 * @returns {Promise<boolean>} True if file is missing front matter
 */
async function hasMissingFrontMatter(filePath) {
  const hasFrontMatter = await hasValidFrontMatterAtStart(filePath);
  return !hasFrontMatter;
}

/**
 * Checks if a file has invalid YAML syntax in front matter
 * @param {string} filePath - Path to the file to check
 * @returns {Promise<boolean>} True if file has invalid YAML syntax
 */
async function hasInvalidYamlSyntax(filePath) {
  try {
    const content = await fs.readFile(filePath, 'utf8');
    if (!content.startsWith('---\n')) {
      return false; // No front matter to validate
    }

    // Extract the front matter (content between the first --- and the next ---)
    const lines = content.split('\n');
    let frontMatterEndIndex = -1;
    for (let i = 1; i < lines.length; i++) {
      if (lines[i].trim() === '---') {
        frontMatterEndIndex = i;
        break;
      }
    }

    if (frontMatterEndIndex === -1) {
      return true; // No closing --- found, so the front matter is incomplete/invalid
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
 * Fixes common YAML formatting issues in frontmatter
 * @param {string} yamlContent - Raw YAML content to fix
 * @returns {string} Fixed YAML content
 */
function fixYamlFrontMatter(yamlContent) {
  // Fix the common issue: keywords: [] followed by list items
  // Pattern: keywords: [] followed by indented list items
  yamlContent = yamlContent.replace(/^(keywords:\s*\[\]\s*\n(?:\s*-.+\n?)+)/m, function(match) {
    // Remove the empty [] and leave just the list items
    return match.replace(/keywords:\s*\[\]\s*\n/, 'keywords:\n');
  });

  return yamlContent;
}

/**
 * Process the parsed YAML object to fix any structural issues
 * @param {Object} parsedYaml - The parsed YAML object
 * @returns {Object} Fixed YAML object
 */
function fixParsedYamlObject(parsedYaml) {
  // If keywords exist and seem to be a comma-separated string rather than a list, convert it
  if (parsedYaml.keywords && typeof parsedYaml.keywords === 'string' && parsedYaml.keywords.includes(',')) {
    // Split the comma-separated string and convert to an array
    parsedYaml.keywords = parsedYaml.keywords.split(',').map(item => item.trim());
  }
  return parsedYaml;
}

/**
 * Parse YAML front matter from a file
 * @param {string} filePath - Path to the file to parse
 * @returns {Promise<object>} Parsed front matter object or empty object if no front matter
 */
async function parseYamlFrontMatter(filePath) {
  try {
    const content = await fs.readFile(filePath, 'utf8');
    if (!content.startsWith('---\n')) {
      return {}; // No front matter to parse
    }

    // Extract the front matter (content between the first --- and the next ---)
    const lines = content.split('\n');
    let frontMatterEndIndex = -1;
    for (let i = 1; i < lines.length; i++) {
      if (lines[i].trim() === '---') {
        frontMatterEndIndex = i;
        break;
      }
    }

    if (frontMatterEndIndex === -1) {
      return {}; // No closing --- found
    }

    // Extract the YAML content
    let yamlContent = lines.slice(1, frontMatterEndIndex).join('\n');

    // Fix common YAML formatting issues before parsing
    yamlContent = fixYamlFrontMatter(yamlContent);

    try {
      // Parse the YAML
      let parsedYaml = yaml.load(yamlContent);
      parsedYaml = fixParsedYamlObject(parsedYaml || {});
      return parsedYaml;
    } catch (yamlError) {
      console.error(`Error parsing YAML for ${filePath}:`, yamlError);
      return {};
    }
  } catch (error) {
    console.error(`Error parsing front matter for ${filePath}:`, error);
    return {};
  }
}

module.exports = {
  hasValidFrontMatterAtStart,
  hasMissingFrontMatter,
  hasInvalidYamlSyntax,
  parseYamlFrontMatter
};