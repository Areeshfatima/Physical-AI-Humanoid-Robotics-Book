const fs = require('fs');
const path = require('path');
const { parseYaml, isValidYaml, quoteYamlValue } = require('./yaml-parser');
const { extractFrontMatter } = require('./frontmatter');
const { createBackup } = require('./backup');

/**
 * Initial skeleton for the front matter repair function
 */

/**
 * Repair the front matter of a markdown file
 * @param {string} filePath - Path to the markdown file to repair
 * @param {object} options - Options for repair
 * @returns {object} - Result of the repair operation
 */
function repairFrontMatter(filePath, options = {}) {
  const { createBackup: shouldCreateBackup = true } = options;
  
  // Create backup if requested
  if (shouldCreateBackup) {
    createBackup(filePath);
  }
  
  // Extract current front matter and content
  const { frontMatter: currentFrontMatter, content, hasFrontMatter } = extractFrontMatter(filePath);
  
  let updatedFrontMatter = {};
  
  if (hasFrontMatter && currentFrontMatter) {
    // If there's existing front matter, try to parse it
    const parsedFrontMatter = parseYaml(currentFrontMatter);
    if (parsedFrontMatter) {
      updatedFrontMatter = parsedFrontMatter;
    }
  }
  
  // Ensure required fields exist
  const fileName = path.basename(filePath, path.extname(filePath));
  
  // Ensure required fields (id and title) exist
  if (!updatedFrontMatter.id) {
    updatedFrontMatter.id = fileName;
  }
  
  if (!updatedFrontMatter.title) {
    // Generate title from filename by capitalizing words and replacing hyphens/underscores
    updatedFrontMatter.title = fileName
      .replace(/[-_]/g, ' ')
      .split(' ')
      .map(word => word.charAt(0).toUpperCase() + word.slice(1))
      .join(' ');
  }
  
  // Apply defensive quoting to string values
  for (const [key, value] of Object.entries(updatedFrontMatter)) {
    if (typeof value === 'string') {
      updatedFrontMatter[key] = quoteYamlValue(value);
    }
  }
  
  // Generate the new front matter string
  const frontMatterLines = ['---'];
  for (const [key, value] of Object.entries(updatedFrontMatter)) {
    frontMatterLines.push(`${key}: ${typeof value === 'string' && value.startsWith('"') ? value : `"${value}"`}`);
  }
  frontMatterLines.push('---', ''); // Extra blank line after front matter
  
  // Combine front matter and content
  const newContent = frontMatterLines.join('\n') + content;
  
  // Write the updated content back to the file
  fs.writeFileSync(filePath, newContent);
  
  return {
    filePath,
    status: 'repaired',
    actionTaken: hasFrontMatter ? 'updated existing front matter' : 'added missing front matter',
    errors: []
  };
}

module.exports = {
  repairFrontMatter
};