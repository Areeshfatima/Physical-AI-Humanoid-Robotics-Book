#!/usr/bin/env node

const fs = require('fs').promises;
const path = require('path');
const { scanDocumentationFiles } = require('./utils/file-scanner');
const {
  hasValidFrontMatterAtStart,
  hasMissingFrontMatter,
  hasInvalidYamlSyntax,
  parseYamlFrontMatter
} = require('./utils/frontmatter');
const { parseYaml, dumpYaml } = require('./utils/yaml-parser');
const { hasRequiredFields, addRequiredFields } = require('./utils/validator');
const { extractContent, reconstructContent } = require('./utils/content-extractor');
const { createBackup } = require('./backup-util');
const { applyDefensiveQuoting, fixYamlFormatting } = require('./utils/formatting');

/**
 * Processes a single markdown file to repair its front matter
 * @param {string} filePath - Path to the markdown file
 * @param {boolean} shouldCreateBackup - Whether to create a backup before modifying
 */
async function processFile(filePath, shouldCreateBackup = true) {
  console.log(`Processing: ${filePath}`);

  try {
    // Read the file content
    const originalContent = await fs.readFile(filePath, 'utf8');

    // Create a backup if requested
    if (shouldCreateBackup) {
      await createBackup(filePath);
    }
    
    // Check if file has valid front matter at start
    const hasValidFrontMatter = await hasValidFrontMatterAtStart(filePath);
    
    let updatedContent = originalContent;
    
    if (await hasMissingFrontMatter(filePath)) {
      // File is missing front matter, add minimal required front matter
      console.log(`  - Adding missing front matter to ${filePath}`);
      
      // Generate id and title based on filename
      const filename = path.basename(filePath);
      const id = filename
        .replace(/\.[^/.]+$/, '') // Remove extension
        .toLowerCase()
        .replace(/[^a-z0-9]+/g, '-') // Replace non-alphanumeric with hyphens
        .replace(/(^-|-$)/g, ''); // Remove leading/trailing hyphens
      
      const title = filename
        .replace(/\.[^/.]+$/, '') // Remove extension
        .replace(/[-_]+/g, ' ') // Replace hyphens/underscores with spaces
        .replace(/\b\w/g, l => l.toUpperCase()); // Capitalize first letter of each word
      
      // Create new front matter with required fields
      const newFrontMatter = { id, title };
      const fixedFrontMatter = fixYamlFormatting(newFrontMatter);
      const quotedFrontMatter = applyDefensiveQuoting(fixedFrontMatter);
      const newYaml = dumpYaml(quotedFrontMatter).trim();
      
      // Reconstruct content with new front matter
      updatedContent = `---\n${newYaml}---\n${originalContent}`;
    } else if (await hasInvalidYamlSyntax(filePath)) {
      // File has invalid YAML syntax, try to repair it
      console.log(`  - Repairing invalid YAML syntax in ${filePath}`);
      
      // Extract the current content
      const { frontMatter: currentFrontMatter, bodyContent } = extractContent(originalContent);
      
      // Try to parse the current front matter (it's invalid, but we might be able to fix it)
      let parsedFrontMatter;
      try {
        parsedFrontMatter = parseYaml(currentFrontMatter);
      } catch (e) {
        console.log(`  - Could not parse invalid front matter, will create new one`);
        parsedFrontMatter = {};
      }
      
      // Ensure required fields are present
      const filename = path.basename(filePath);
      const updatedFrontMatter = addRequiredFields(parsedFrontMatter, filename);
      const fixedFrontMatter = fixYamlFormatting(updatedFrontMatter);
      const quotedFrontMatter = applyDefensiveQuoting(fixedFrontMatter);

      // Generate properly formatted YAML
      const newYaml = dumpYaml(quotedFrontMatter);
      
      // Reconstruct the content
      updatedContent = reconstructContent(newYaml, bodyContent);
    } else {
      // File has valid front matter, check if it has required fields
      const currentFrontMatter = await parseYamlFrontMatter(filePath);
      const requiredFieldsResult = hasRequiredFields(currentFrontMatter);
      
      if (!requiredFieldsResult.valid) {
        console.log(`  - Adding missing required fields to ${filePath}`);
        
        // Add missing required fields
        const filename = path.basename(filePath);
        const updatedFrontMatter = addRequiredFields(currentFrontMatter, filename);
        const fixedFrontMatter = fixYamlFormatting(updatedFrontMatter);
        const quotedFrontMatter = applyDefensiveQuoting(fixedFrontMatter);

        // Extract body content
        const { bodyContent } = extractContent(originalContent);

        // Generate properly formatted YAML
        const newYaml = dumpYaml(quotedFrontMatter);

        // Reconstruct the content
        updatedContent = reconstructContent(newYaml, bodyContent);
      } else {
        // Even if required fields are present, check for proper formatting
        const filename = path.basename(filePath);
        let updatedFrontMatter = { ...currentFrontMatter };

        // Apply formatting fixes regardless
        const fixedFrontMatter = fixYamlFormatting(updatedFrontMatter);
        const quotedFrontMatter = applyDefensiveQuoting(fixedFrontMatter);

        // Check if any formatting changes were made
        if (JSON.stringify(updatedFrontMatter) !== JSON.stringify(quotedFrontMatter)) {
          console.log(`  - Fixing formatting in front matter of ${filePath}`);

          // Extract body content
          const { bodyContent } = extractContent(originalContent);

          // Generate properly formatted YAML
          const newYaml = dumpYaml(quotedFrontMatter).trim();

          // Reconstruct the content
          updatedContent = reconstructContent(newYaml, bodyContent);
        } else {
          console.log(`  - File already has valid front matter: ${filePath}`);
          // File is already valid with proper formatting, no changes needed
          return { filePath, status: 'already_valid', changes: [] };
        }
      }
    }
    
    // Write the updated content back to the file
    await fs.writeFile(filePath, updatedContent);
    
    console.log(`  - Updated: ${filePath}`);
    return { 
      filePath, 
      status: 'fixed', 
      changes: ['Repaired front matter'] 
    };
  } catch (error) {
    console.error(`Error processing ${filePath}:`, error);
    return { 
      filePath, 
      status: 'error', 
      changes: [], 
      errors: [error.message] 
    };
  }
}

/**
 * Main function to repair front matter in all documentation files
 * @param {string} docsDir - Directory containing documentation files (default: 'my-book/docs/')
 * @param {Object} options - Options for processing
 * @param {boolean} options.progress - Whether to show progress updates
 */
async function repairFrontMatter(docsDir = 'my-book/docs/', options = { progress: true }) {
  if (options.progress) {
    console.log(`Scanning documentation files in ${docsDir}...`);
  }

  // Get all documentation files
  const files = await scanDocumentationFiles(docsDir);
  if (options.progress) {
    console.log(`Found ${files.length} documentation files`);
  }

  const results = {
    total: files.length,
    fixed: 0,
    valid: 0,
    errors: 0,
    results: [],
    processed: 0
  };

  // Process each file
  for (const file of files) {
    if (options.progress) {
      results.processed++;
      console.log(`\n[${results.processed}/${files.length}] Processing: ${file}`);
    }

    const result = await processFile(file, true); // Create backups
    results.results.push(result);

    switch (result.status) {
      case 'fixed':
        results.fixed++;
        if (options.progress) console.log(`  - Status: FIXED`);
        break;
      case 'already_valid':
        results.valid++;
        if (options.progress) console.log(`  - Status: ALREADY_VALID`);
        break;
      case 'error':
        results.errors++;
        if (options.progress) console.log(`  - Status: ERROR - ${result.errors ? result.errors[0] : 'Unknown error'}`);
        break;
    }

    if (options.progress && results.processed % 10 === 0) {
      console.log(`  Progress: ${results.processed}/${files.length} files processed`);
    }
  }

  if (options.progress) {
    console.log('\n--- Processing Summary ---');
    console.log(`Total files processed: ${results.total}`);
    console.log(`Files fixed: ${results.fixed}`);
    console.log(`Files already valid: ${results.valid}`);
    console.log(`Files with errors: ${results.errors}`);
  }

  return results;
}

// For testing purposes, run immediately if this file is executed directly
if (require.main === module) {
  const docsDir = process.argv[2] || 'my-book/docs/';
  const options = { progress: true };
  repairFrontMatter(docsDir, options)
    .then(results => {
      console.log('\nRepair process completed.');
      process.exit(0);
    })
    .catch(error => {
      console.error('Repair process failed:', error);
      process.exit(1);
    });
}

module.exports = {
  processFile,
  repairFrontMatter
};