#!/usr/bin/env node

/**
 * Enhanced front matter validator and fixer for Docusaurus v3
 * 
 * This script specifically addresses the "Cannot destructure property 'frontMatter'" error
 * by ensuring all markdown files in the docs directory have properly formatted YAML front matter
 */

const fs = require('fs');
const path = require('path');
const yaml = require('js-yaml');

// Check if required dependencies are available
try {
  require('js-yaml');
} catch (error) {
  console.error('Missing required dependency: js-yaml');
  console.error('Install it with: npm install js-yaml');
  process.exit(1);
}

/**
 * Extract front matter from a markdown file
 * @param {string} filePath - Path to the markdown file
 * @returns {object} - Object with frontMatter, content, and hasFrontMatter properties
 */
function extractFrontMatter(filePath) {
  const content = fs.readFileSync(filePath, 'utf8');
  
  if (!content.startsWith('---')) {
    return {
      frontMatter: null,
      content: content,
      hasFrontMatter: false
    };
  }

  // Find the end of the front matter (second occurrence of ---)
  const lines = content.split(/\r?\n/);
  if (lines.length < 2) {
    return {
      frontMatter: null,
      content: content,
      hasFrontMatter: false
    };
  }

  let frontMatterEndIndex = -1;
  for (let i = 1; i < lines.length; i++) {
    if (lines[i].trim() === '---') {
      frontMatterEndIndex = i;
      break;
    }
  }

  if (frontMatterEndIndex === -1) {
    // Unclosed front matter block
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
 * Attempt to repair common YAML syntax errors
 * @param {string} yamlStr - The YAML string with potential errors
 * @returns {string} - The repaired YAML string
 */
function repairYamlSyntax(yamlStr) {
  if (!yamlStr || typeof yamlStr !== 'string') {
    return '';
  }

  let repairedYaml = yamlStr;

  // 1. Fix unquoted values that contain colons followed by spaces (common cause of parsing errors)
  repairedYaml = repairedYaml.replace(/^(\s*[a-z_][a-z0-9_-]*:)\s+([^"'#]*: ?[^"'#]*)$/gim, '$1 "$2"');

  // 2. Fix unquoted strings that look like booleans
  repairedYaml = repairedYaml.replace(/^(\s*[a-z_][a-z0-9_-]*:)\s+(true|false|yes|no|null|on|off)(?=\s|$)/gim, '$1 "$2"');

  // 3. Fix unquoted values containing special characters
  repairedYaml = repairedYaml.replace(/^(\s*[a-z_][a-z0-9_-]*:)\s+([^"'#]*[<>"'&@`{}[\]#|>!][^"'#]*)$/gim, '$1 "$2"');

  // 4. Fix values that start with numbers (which YAML might parse as numbers)
  repairedYaml = repairedYaml.replace(/^(\s*[a-z_][a-z0-9_-]*:)\s+([0-9].*)$/gim, '$1 "$2"');

  // 5. Handle multi-line descriptions that start with > or |
  repairedYaml = repairedYaml.replace(/^(\s*description:\s*)(>|-|\|)(\s*.*)/gim, '$1$2$3');

  return repairedYaml;
}

/**
 * Validate that front matter has required fields for Docusaurus
 * @param {object} parsedFrontMatter - The parsed front matter object
 * @returns {object} - Object with validation results
 */
function validateRequiredFields(parsedFrontMatter) {
  const errors = [];
  
  if (!parsedFrontMatter) {
    errors.push('Front matter could not be parsed');
    return { isValid: false, errors };
  }

  if (!parsedFrontMatter.id) {
    errors.push("Missing required field: 'id'");
  }

  if (!parsedFrontMatter.title) {
    errors.push("Missing required field: 'title'");
  }

  return {
    isValid: errors.length === 0,
    errors
  };
}

/**
 * Generate a default front matter for a file if needed
 * @param {string} filePath - Path to the markdown file
 * @returns {object} - A default front matter object
 */
function generateDefaultFrontMatter(filePath) {
  const fileName = path.basename(filePath, path.extname(filePath));
  const dirName = path.basename(path.dirname(filePath));
  
  // Generate a title from the filename
  const title = fileName
    .replace(/[-_]/g, ' ')
    .split(' ')
    .map(word => word.charAt(0).toUpperCase() + word.slice(1))
    .join(' ');

  // Create a unique ID combining directory and filename
  const id = `${dirName}-${fileName}`.replace(/[^a-zA-Z0-9-_]/g, '-').toLowerCase();

  return {
    title: title,
    id: id
  };
}

/**
 * Fix the front matter of a markdown file
 * @param {string} filePath - Path to the markdown file to fix
 * @returns {object} - Result object with status and details
 */
function fixFrontMatter(filePath) {
  try {
    const { frontMatter, content, hasFrontMatter } = extractFrontMatter(filePath);

    let parsedFrontMatter = null;
    let repairedYaml = frontMatter;
    
    if (hasFrontMatter && frontMatter) {
      try {
        // First, try to parse the existing front matter
        parsedFrontMatter = yaml.load(frontMatter);
      } catch (parseError) {
        // If parsing fails, try to repair the YAML syntax and parse again
        console.log(`  - Attempting to repair YAML syntax in ${filePath}`);
        repairedYaml = repairYamlSyntax(frontMatter);
        
        try {
          parsedFrontMatter = yaml.load(repairedYaml);
        } catch (repairedParseError) {
          // If repaired YAML still can't be parsed, we'll create new front matter
          console.log(`  - Failed to parse repaired YAML in ${filePath}, will recreate`);
          parsedFrontMatter = null;
        }
      }
    }

    // If we still don't have parsed front matter, create default
    if (!parsedFrontMatter) {
      console.log(`  - Creating default front matter for ${filePath}`);
      parsedFrontMatter = generateDefaultFrontMatter(filePath);
    }

    // Validate required fields
    const validation = validateRequiredFields(parsedFrontMatter);
    if (!validation.isValid) {
      console.log(`  - Adding missing required fields to ${filePath}:`, validation.errors);
      
      // Add default values for missing required fields
      if (!parsedFrontMatter.id) {
        parsedFrontMatter.id = path.basename(filePath, path.extname(filePath));
      }
      
      if (!parsedFrontMatter.title) {
        parsedFrontMatter.title = path.basename(filePath, path.extname(filePath))
          .replace(/[-_]/g, ' ')
          .split(' ')
          .map(word => word.charAt(0).toUpperCase() + word.slice(1))
          .join(' ');
      }
    }

    // Ensure proper formatting for string fields that could cause issues
    for (const [key, value] of Object.entries(parsedFrontMatter)) {
      if (typeof value === 'string') {
        // Quote strings that contain special characters that could cause YAML parsing issues
        if (/[<>"'&@`{}[\]#|>!]/.test(value) || 
            value.includes(': ') || 
            value.startsWith('http') || 
            /^\d/.test(value) || 
            /^(true|false|yes|no|null|on|off)$/i.test(value)) {
          // Already handled by YAML dump which quotes these automatically
        }
      }
    }

    // Generate the properly formatted YAML front matter
    const newYaml = yaml.dump(parsedFrontMatter, { 
      lineWidth: -1,  // Don't wrap lines
      noRefs: true,   // Don't use references
      skipInvalid: false  // Don't skip invalid values
    });

    // Combine the new front matter with the content
    const fixedContent = `---\n${newYaml}---\n\n${content}`;

    // Write the fixed content back to the file
    fs.writeFileSync(filePath, fixedContent);

    return {
      success: true,
      filePath,
      action: hasFrontMatter ? 'repaired existing front matter' : 'added missing front matter',
      issuesFound: validation.errors.length
    };
  } catch (error) {
    return {
      success: false,
      filePath,
      error: error.message,
      action: 'failed to fix'
    };
  }
}

/**
 * Process all markdown files in a directory
 * @param {string} dirPath - Directory path to process
 */
function processDirectory(dirPath) {
  const allFiles = [];
  
  function walkDirectory(currentPath) {
    const items = fs.readdirSync(currentPath);
    
    for (const item of items) {
      const fullPath = path.join(currentPath, item);
      const stat = fs.statSync(fullPath);
      
      if (stat.isDirectory()) {
        // Skip node_modules and other common non-doc directories
        if (item !== 'node_modules' && item !== '.git' && item !== '.docusaurus') {
          walkDirectory(fullPath);
        }
      } else if (path.extname(fullPath) === '.md') {
        allFiles.push(fullPath);
      }
    }
  }
  
  walkDirectory(dirPath);
  return allFiles;
}

// Main execution
function main() {
  // Default to docs directory if no argument provided
  const targetDir = process.argv[2] || './docs';
  
  if (!fs.existsSync(targetDir)) {
    console.error(`Directory does not exist: ${targetDir}`);
    process.exit(1);
  }
  
  console.log(`Scanning directory: ${targetDir}`);
  const markdownFiles = processDirectory(targetDir);
  
  console.log(`Found ${markdownFiles.length} markdown files to process.`);
  
  const results = [];
  let successCount = 0;
  let failureCount = 0;
  
  for (const file of markdownFiles) {
    console.log(`Processing: ${file}`);
    const result = fixFrontMatter(file);
    results.push(result);
    
    if (result.success) {
      console.log(`  - ${result.action}`);
      successCount++;
    } else {
      console.error(`  - Error: ${result.error}`);
      failureCount++;
    }
  }
  
  // Summary
  console.log('\n--- Processing Summary ---');
  console.log(`Total files processed: ${results.length}`);
  console.log(`Successfully fixed: ${successCount}`);
  console.log(`Failed to fix: ${failureCount}`);
  
  if (failureCount > 0) {
    console.log('\nFiles that failed to process:');
    results
      .filter(r => !r.success)
      .forEach(r => console.log(`  - ${r.filePath}: ${r.error}`));
  }
  
  console.log('\nValidation completed. You can now build your Docusaurus site.');
}

// Run the script
if (require.main === module) {
  main();
}