#!/usr/bin/env node

const fs = require('fs');
const path = require('path');
const { scanMarkdownFiles } = require('./utils/file-scanner');
const { extractFrontMatter } = require('./utils/frontmatter');
const { parseYaml, quoteYamlValue } = require('./utils/yaml-parser');
const { createBackup } = require('./utils/backup');
const { hasMissingRequiredFields } = require('./utils/validator');

/**
 * Main repair script for fixing markdown front matter issues
 */

function repairFrontMatter(filePath) {
  // Create backup
  createBackup(filePath);

  // Extract current front matter and content
  const { frontMatter: currentFrontMatter, content, hasFrontMatter } = extractFrontMatter(filePath);
  let updatedFrontMatter = {};

  if (hasFrontMatter && currentFrontMatter) {
    // If there's existing front matter, try to parse it
    let parsedFrontMatter = parseYaml(currentFrontMatter);

    if (!parsedFrontMatter) {
      // If the front matter can't be parsed, try to repair common syntax errors
      const repairedFrontMatter = repairYamlSyntax(currentFrontMatter);
      parsedFrontMatter = parseYaml(repairedFrontMatter);

      if (parsedFrontMatter) {
        // Use the repaired front matter if parsing succeeded
        updatedFrontMatter = parsedFrontMatter;
      } else {
        // If still can't parse after repair attempts, treat as invalid and start fresh
        console.warn(`Could not repair invalid front matter in ${filePath}, recreating...`);
      }
    } else {
      // If parsed successfully, use the parsed front matter
      updatedFrontMatter = parsedFrontMatter;
    }
  }
  
  // Generate filename-based id and title if missing
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
  
  // Generate the new front matter string using yaml.dump for proper formatting
  const yaml = require('js-yaml');
  const frontMatterYAML = yaml.dump(updatedFrontMatter);

  // Combine front matter and content properly
  const newContent = `---\n${frontMatterYAML}---\n\n${content}`;
  
  // Write the updated content back to the file
  fs.writeFileSync(filePath, newContent);
  
  const missingFieldsCheck = hasMissingRequiredFields(filePath);
  return {
    filePath,
    status: 'repaired',
    actionTaken: hasFrontMatter ? 'updated existing front matter' : 'added missing front matter',
    missingFields: missingFieldsCheck.missingFields,
    errors: []
  };
}

/**
 * Attempt to repair common YAML syntax errors
 * @param {string} yamlStr - The YAML string with potential errors
 * @returns {string} - The repaired YAML string
 */
function repairYamlSyntax(yamlStr) {
  // Handle common YAML issues:

  // 1. Fix unquoted values that contain colons followed by spaces
  yamlStr = yamlStr.replace(/^(\s*[a-zA-Z_][a-zA-Z0-9_-]*:)\s+([^"'][^#]*):([^#]*)$/gm, '$1 "$2:$3"');

  // 2. Fix unquoted values that begin with special characters
  yamlStr = yamlStr.replace(/^(\s*[a-zA-Z_][a-zA-Z0-9_-]*:)\s+([!&*]>|!important|!default)/gm, '$1 "$2"');

  // 3. Fix unquoted values containing special symbols
  yamlStr = yamlStr.replace(/^(\s*[a-zA-Z_][a-zA-Z0-9_-]*:)\s+([^"'#]*[<>"'&@`{}[\]]+[^"'#]*)$/gm, '$1 "$2"');

  // 4. Fix unquoted boolean values that are lowercase
  yamlStr = yamlStr.replace(/^(\s*[a-zA-Z_][a-zA-Z0-9_-]*:)\s+(true|false)$/gm, '$1 $2');

  // 5. Fix unquoted values with leading/trailing spaces
  yamlStr = yamlStr.replace(/^(\s*[a-zA-Z_][a-zA-Z0-9_-]*:)\s+"?\s+([^"]*)\s+"?$/gm, '$1 "$2"');

  // 6. Fix values that contain # without being properly quoted
  yamlStr = yamlStr.replace(/^(\s*[a-zA-Z_][a-zA-Z0-9_-]*:)\s+([^"#]*)#(.*)$/gm, '$1 "$2#$3"');

  return yamlStr;
}

// Process command line arguments
const args = process.argv.slice(2);
let targetDir = 'my-book/docs/';

if (args.length > 0) {
  targetDir = args[0];
}

// Scan and process all markdown files
const markdownFiles = scanMarkdownFiles(targetDir);
const results = [];

console.log(`Found ${markdownFiles.length} markdown files to process.`);

for (const file of markdownFiles) {
  try {
    console.log(`Processing: ${file}`);
    const result = repairFrontMatter(file);
    results.push(result);
    console.log(`  - ${result.actionTaken}`);
  } catch (error) {
    console.error(`Error processing ${file}:`, error.message);
    results.push({
      filePath: file,
      status: 'failed',
      actionTaken: 'none',
      errors: [error.message]
    });
  }
}

// Output summary
console.log('\n--- Processing Summary ---');
console.log(`Total files processed: ${results.length}`);
console.log(`Successfully repaired: ${results.filter(r => r.status === 'repaired').length}`);
console.log(`Failed: ${results.filter(r => r.status === 'failed').length}`);

// Check if npm start would work now by validating all files
console.log('\nValidating all files for Docusaurus compatibility...');
let validCount = 0;
for (const file of markdownFiles) {
  const { hasValidFrontMatterYAML } = require('./utils/validator');
  if (hasValidFrontMatterYAML(file)) {
    validCount++;
  }
}

console.log(`Files with valid YAML front matter: ${validCount}/${markdownFiles.length}`);