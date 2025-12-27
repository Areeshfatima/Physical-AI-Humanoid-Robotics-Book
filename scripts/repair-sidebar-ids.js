#!/usr/bin/env node

const fs = require('fs').promises;
const path = require('path');
const glob = require('glob');
const { scanDocumentationFiles, isHiddenFile } = require('../src/utils/file-scanner');
const {
  hasValidFrontMatterAtStart,
  hasMissingFrontMatter,
  hasInvalidYamlSyntax,
  parseYamlFrontMatter
} = require('../src/utils/frontmatter-processor');
const { generateIdFromFilename, generateTitleFromFilename, sanitizeId, IdManager } = require('../src/utils/id-generator');
const { resolveIdConflict, ConflictResolver } = require('../src/utils/conflict-resolver');
const {
  hasRequiredFields,
  isValidId,
  isValidTitle,
  isValidSidebarPosition,
  validateFrontMatter
} = require('../src/utils/validator');
const { generateComplexSidebarFromTree, writeSidebarToFile } = require('../src/utils/sidebar-generator');

/**
 * Creates a backup of a file before modification
 * @param {string} filePath - Path to the file to backup
 * @returns {Promise<string>} Path to the created backup file
 */
async function createBackup(filePath) {
  const backupDir = 'backups/front-matter-fix';
  const fileName = path.basename(filePath);
  const fileDir = path.dirname(filePath);
  const relativeDir = path.relative('.', fileDir);
  const backupPath = path.join(backupDir, relativeDir, fileName + '.bak');
  
  // Ensure backup directory exists
  await fs.mkdir(path.dirname(backupPath), { recursive: true });
  
  // Read the original file content
  const originalContent = await fs.readFile(filePath, 'utf8');
  
  // Write content to backup location
  await fs.writeFile(backupPath, originalContent);
  
  console.log(`Backup created: ${filePath} -> ${backupPath}`);
  return backupPath;
}

/**
 * Serialize a front matter object into YAML format
 * @param {Object} frontMatter - The front matter object to serialize
 * @returns {string} The serialized YAML string with proper formatting
 */
function serializeFrontMatter(frontMatter) {
  let yamlStr = '';
  for (const [key, value] of Object.entries(frontMatter)) {
    if (Array.isArray(value)) {
      // For arrays, format as YAML list
      yamlStr += `${key}:\n`;
      for (const item of value) {
        yamlStr += `  - ${item}\n`;
      }
    } else if (typeof value === 'string') {
      // For strings, quote them to handle special characters
      yamlStr += `${key}: "${value}"\n`;
    } else {
      yamlStr += `${key}: ${value}\n`;
    }
  }
  return yamlStr;
}

/**
 * Process a single markdown file to repair its front matter
 * @param {string} filePath - Path to the markdown file
 * @param {IdManager} idManager - Manager for tracking unique IDs
 * @param {boolean} shouldCreateBackup - Whether to create a backup before modifying
 * @param {boolean} dryRun - Whether to run without making changes (just preview)
 * @param {boolean} verbose - Whether to output detailed information
 */
async function processFile(filePath, idManager, shouldCreateBackup = true, dryRun = false, verbose = false) {
  if (verbose) {
    console.log(`Processing: ${filePath}`);
  }

  try {
    // Read the file content
    const originalContent = await fs.readFile(filePath, 'utf8');

    // Check if file has valid front matter at start
    const hasValidFrontMatter = await hasValidFrontMatterAtStart(filePath);
    const missingFrontMatter = await hasMissingFrontMatter(filePath);
    const invalidYamlSyntax = await hasInvalidYamlSyntax(filePath);

    let updatedContent = originalContent;
    let hasChanges = false;

    if (missingFrontMatter) {
      if (verbose) {
        console.log(`  - Missing front matter detected in ${filePath}`);
      }

      // If performing a dry run, just report what would be done
      if (dryRun) {
        console.log(`  - Would create new front matter for ${filePath}`);
        return { filePath, status: 'would_fix', changes: ['Missing front matter would be added'] };
      }

      // Create new front matter with required fields
      const fileName = path.basename(filePath);
      const id = idManager.addId(generateIdFromFilename(fileName));
      const title = generateTitleFromFilename(fileName);
      const sidebarPosition = 0; // Default to 0 if not specified elsewhere

      // Create new front matter object
      const newFrontMatter = { id, title, sidebar_position: sidebarPosition };

      // Construct the new content with front matter
      let updatedContent = `---\n${serializeFrontMatter(newFrontMatter)}---\n${originalContent}`;
      let hasChanges = true;
    } else if (invalidYamlSyntax) {
      if (verbose) {
        console.log(`  - Invalid YAML syntax detected in ${filePath}`);
      }

      if (dryRun) {
        console.log(`  - Would fix invalid YAML syntax in ${filePath}`);
        return { filePath, status: 'would_fix', changes: ['Invalid YAML syntax would be fixed'] };
      }

      // Parse current content to extract body
      const parsedResult = await parseYamlFrontMatter(filePath);
      const contentParts = originalContent.split('---');
      const currentContent = contentParts.length > 2 ? contentParts.slice(2).join('---').trim() : ''; // Everything after the second ---

      // Ensure required fields are present
      const fileName = path.basename(filePath);
      let updatedFrontMatter = { ...parsedResult };

      // Add or fix required fields if missing or invalid
      if (!updatedFrontMatter.id || !isValidId(updatedFrontMatter.id)) {
        updatedFrontMatter.id = idManager.addId(generateIdFromFilename(fileName));
      } else {
        // If ID is already present and valid, make sure to register it in the ID manager to prevent conflicts
        idManager.addId(updatedFrontMatter.id);
      }

      if (!updatedFrontMatter.title || !isValidTitle(updatedFrontMatter.title)) {
        updatedFrontMatter.title = generateTitleFromFilename(fileName);
      }

      if (!updatedFrontMatter.sidebar_position || !isValidSidebarPosition(updatedFrontMatter.sidebar_position)) {
        updatedFrontMatter.sidebar_position = 0; // Default position
      }

      // Reconstruct content with fixed front matter
      updatedContent = `---\n${serializeFrontMatter(updatedFrontMatter)}---\n${currentContent}`;
      hasChanges = true;
    } else {
      // File has valid front matter, check if it has required fields
      const currentFrontMatter = await parseYamlFrontMatter(filePath);
      const requiredFieldsResult = hasRequiredFields(currentFrontMatter);

      if (!requiredFieldsResult.valid) {
        if (verbose) {
          console.log(`  - Missing required fields detected in ${filePath}`);
        }

        if (dryRun) {
          console.log(`  - Would add missing required fields to ${filePath}`);
          return { filePath, status: 'would_fix', changes: ['Missing required fields would be added'] };
        }

        // Add missing required fields
        const fileName = path.basename(filePath);
        let updatedFrontMatter = { ...currentFrontMatter };

        if (!updatedFrontMatter.id) {
          updatedFrontMatter.id = idManager.addId(generateIdFromFilename(fileName));
        } else {
          // If ID exists, make sure it's registered in the ID manager to prevent conflicts
          idManager.addId(updatedFrontMatter.id);
        }

        if (!updatedFrontMatter.title) {
          updatedFrontMatter.title = generateTitleFromFilename(fileName);
        }

        if (!updatedFrontMatter.sidebar_position) {
          updatedFrontMatter.sidebar_position = 0; // Default position
        }

        // Extract body content (everything after the front matter)
        const contentParts = originalContent.split('---');
        const bodyContent = contentParts.length > 2 ? contentParts.slice(2).join('---').trim() : '';

        // Reconstruct the content
        updatedContent = `---\n${serializeFrontMatter(updatedFrontMatter)}---\n${bodyContent}`;
        hasChanges = true;
      } else {
        // File is already valid, check for uniqueness of IDs and validate
        if (currentFrontMatter.id) {
          // Register the ID in the manager to track for potential future conflicts
          idManager.addId(currentFrontMatter.id);
        }

        // Check if the file needs to be reformatted with proper YAML serialization
        // Extract body content (everything after the front matter)
        const contentParts = originalContent.split('---');
        const bodyContent = contentParts.length > 2 ? contentParts.slice(2).join('---').trim() : '';

        // Create properly formatted front matter content
        const formattedFrontMatterContent = `---\n${serializeFrontMatter(currentFrontMatter)}---\n${bodyContent}`;

        // Compare with original content to see if reformatting is needed
        if (originalContent !== formattedFrontMatterContent) {
          if (verbose) {
            console.log(`  - File has valid front matter but needs reformatting: ${filePath}`);
          }

          if (dryRun) {
            return { filePath, status: 'would_fix', changes: ['Front matter would be reformatted'] };
          }

          // Create backup if requested
          if (shouldCreateBackup) {
            await createBackup(filePath);
          }

          // Write the properly formatted content back to the file
          await fs.writeFile(filePath, formattedFrontMatterContent);

          if (verbose) {
            console.log(`  - Reformatted: ${filePath}`);
          }

          return { filePath, status: 'fixed', changes: ['Front matter reformatted'] };
        } else {
          if (verbose) {
            console.log(`  - File already has valid and properly formatted front matter: ${filePath}`);
          }

          return { filePath, status: 'already_valid', changes: [] };
        }
      }
    }

    // If changes were made and not in dry run mode
    if (hasChanges && !dryRun) {
      // Create backup if requested
      if (shouldCreateBackup) {
        await createBackup(filePath);
      }

      // Write the updated content back to the file
      await fs.writeFile(filePath, updatedContent);

      if (verbose) {
        console.log(`  - Updated: ${filePath}`);
      }

      return { filePath, status: 'fixed', changes: ['Front matter repaired'] };
    } else if (hasChanges && dryRun) {
      return { filePath, status: 'would_fix', changes: ['Would fix front matter'] };
    }

    return { filePath, status: 'processed', changes: [] };
  } catch (error) {
    console.error(`Error processing ${filePath}:`, error);
    return { filePath, status: 'error', changes: [], error: error.message };
  }
}

/**
 * Main repair function that coordinates the fixing of front matter and sidebar issues
 */
async function repairSidebarIds(options = {}) {
  const defaultOptions = {
    docsDir: 'my-book/docs/',
    sidebarOutput: 'my-book/sidebars.js',
    dryRun: false,
    verbose: false,
    backup: true
  };

  const config = { ...defaultOptions, ...options };

  if (config.verbose) {
    console.log('Starting repair of Docusaurus sidebar and document IDs...');
    console.log(`Options:`, config);
  }

  console.log(`Scanning documentation files in ${config.docsDir}...`);

  // Get all documentation files
  const allFiles = await scanDocumentationFiles(config.docsDir);

  // Filter out hidden files (starting with underscore)
  const files = allFiles.filter(file => !isHiddenFile(file));

  console.log(`Found ${files.length} documentation files to process`);

  // Initialize ID manager to track unique IDs
  const idManager = new IdManager();

  const results = {
    total: files.length,
    fixed: 0,
    would_fix: 0,
    valid: 0,
    errors: 0,
    results: []
  };

  // Process each file
  for (const file of files) {
    if (config.verbose) {
      console.log(`Processing (${results.results.length + 1}/${files.length}): ${file}`);
    }

    const result = await processFile(file, idManager, config.backup, config.dryRun, config.verbose);
    results.results.push(result);

    switch (result.status) {
      case 'fixed':
        results.fixed++;
        break;
      case 'would_fix':
        results.would_fix++;
        break;
      case 'already_valid':
        results.valid++;
        break;
      case 'error':
        results.errors++;
        break;
    }
  }

  // Generate and write the updated sidebar if not in dry run mode
  if (!config.dryRun) {
    if (config.verbose) {
      console.log('Generating updated sidebar configuration...');
    }

    const sidebarConfig = await generateComplexSidebarFromTree(config.docsDir);

    // Write to the JavaScript sidebar file
    await writeSidebarToFile(sidebarConfig, config.sidebarOutput);
    if (config.verbose) {
      console.log(`Sidebar configuration written to ${config.sidebarOutput}`);
    }

    // Write to the TypeScript sidebar file as well
    const tsSidebarOutput = config.sidebarOutput.replace(/\.js$/, '.ts');
    await writeSidebarToFile(sidebarConfig, tsSidebarOutput);
    if (config.verbose) {
      console.log(`Sidebar configuration written to ${tsSidebarOutput}`);
    }

  } else {
    console.log('Would generate updated sidebar configuration (dry run mode)');
  }

  console.log('\n--- Processing Summary ---');
  console.log(`Total files processed: ${results.total}`);
  if (config.dryRun) {
    console.log(`Files that would be fixed: ${results.would_fix}`);
    console.log(`Files that are already valid: ${results.valid}`);
  } else {
    console.log(`Files fixed: ${results.fixed}`);
    console.log(`Files already valid: ${results.valid}`);
  }
  console.log(`Files with errors: ${results.errors}`);

  return results;
}

// Export the function for use as a module
module.exports = {
  repairSidebarIds
};

// If this script is run directly, execute the repair function
if (require.main === module) {
  const args = process.argv.slice(2);
  const options = {};

  // Parse command line arguments
  for (let i = 0; i < args.length; i++) {
    if (args[i] === '--docs-dir' && args[i + 1]) {
      options.docsDir = args[i + 1];
      i++; // Skip the next argument as it's the value
    } else if (args[i] === '--sidebar-output' && args[i + 1]) {
      options.sidebarOutput = args[i + 1];
      i++;
    } else if (args[i] === '--dry-run') {
      options.dryRun = true;
    } else if (args[i] === '--verbose') {
      options.verbose = true;
    } else if (args[i] === '--no-backup') {
      options.backup = false;
    }
  }

  // Execute the repair function with parsed options
  repairSidebarIds(options)
    .then(result => {
      console.log('Repair completed.');
      process.exit(0);
    })
    .catch(error => {
      console.error('Repair failed:', error);
      process.exit(1);
    });
}