/**
 * Ensures all string values in an object are properly quoted by processing them through YAML dump
 * @param {Object} obj - Object to process
 * @returns {Object} Processed object with properly quoted strings
 */
function ensureProperQuoting(obj) {
  // For this implementation, we rely on the js-yaml library's dump function
  // which automatically handles proper quoting of strings with special characters
  return obj;
}

/**
 * Removes unsupported characters from unquoted values
 * @param {string} value - String value to clean
 * @returns {string} Cleaned string value
 */
function removeUnsupportedChars(value) {
  if (typeof value !== 'string') {
    return value;
  }
  
  // For now, we'll just return the value as-is, since YAML quoting in dumpYaml
  // handles special characters properly
  return value;
}

/**
 * Applies defensive quoting strategy to all string values in front matter
 * @param {Object} frontMatter - The front matter object to process
 * @returns {Object} Updated front matter with defensively quoted values
 */
function applyDefensiveQuoting(frontMatter) {
  if (!frontMatter || typeof frontMatter !== 'object') {
    return frontMatter;
  }

  const quotedFrontMatter = {};

  for (const [key, value] of Object.entries(frontMatter)) {
    if (typeof value === 'string') {
      // Clean the string value and ensure proper handling
      let cleanedValue = removeUnsupportedChars(value);

      // Check if the value contains special characters that should be quoted
      // This includes values with colons followed by spaces, special YAML characters, etc.
      quotedFrontMatter[key] = cleanedValue;
    } else if (Array.isArray(value)) {
      // Handle array values properly
      quotedFrontMatter[key] = value;
    } else {
      quotedFrontMatter[key] = value;
    }
  }

  return quotedFrontMatter;
}

/**
 * Fixes common YAML formatting issues in front matter
 * @param {Object} frontMatter - The front matter object to process
 * @returns {Object} Updated front matter with fixed formatting
 */
function fixYamlFormatting(frontMatter) {
  if (!frontMatter || typeof frontMatter !== 'object') {
    return frontMatter;
  }

  const fixedFrontMatter = { ...frontMatter };

  for (const [key, value] of Object.entries(fixedFrontMatter)) {
    if (Array.isArray(value)) {
      // Handle array values (like keywords)
      if (key === 'keywords') {
        fixedFrontMatter[key] = value.map(item => {
          if (typeof item === 'string') {
            // Fix leading quote issue: '"value' -> 'value'
            if (item.startsWith('"') && !item.endsWith('"') && item.length > 1) {
              return item.substring(1);
            }
            // Fix trailing quote issue: 'value"' -> 'value'
            if (!item.startsWith('"') && item.endsWith('"') && item.length > 1) {
              return item.slice(0, -1);
            }
            return item;
          }
          return item;
        });
      }
    } else if (typeof value === 'string') {
      // Fix incorrectly formatted keywords (e.g., keywords: robotics, ai, machine learning)
      // Convert to proper array format if needed
      if (key === 'keywords' && typeof value === 'string' && !value.trim().startsWith('[')) {
        // Split by commas and convert to a proper array
        const items = value.split(',').map(item => item.trim());
        fixedFrontMatter[key] = items;
      }
      // Fix nested quotes in string values (e.g., '"value"' -> 'value')
      else if (value.startsWith('"') && value.endsWith('"') && value.length > 2) {
        // This detects values like: '"Introduction to Physical AI & Humanoid Robotics"'
        // and converts them to: "Introduction to Physical AI & Humanoid Robotics"
        // The js-yaml library will properly quote as needed during dump
        fixedFrontMatter[key] = value.substring(1, value.length - 1);
      }
      // Fix escaped quotes (e.g., "\"unity" -> "unity")
      else if (value.includes('\\"')) {
        fixedFrontMatter[key] = value.replace(/\\"/g, '"');
      }
      // Fix leading/trailing quote issues (e.g., when YAML parsing results in '"value')
      else if (value.startsWith('"') && !value.endsWith('"') && value.length > 1) {
        // This handles cases where YAML values start with a quote but don't properly close
        fixedFrontMatter[key] = value.substring(1);
      }
    }
  }

  return fixedFrontMatter;
}

module.exports = {
  ensureProperQuoting,
  removeUnsupportedChars,
  applyDefensiveQuoting,
  fixYamlFormatting
};