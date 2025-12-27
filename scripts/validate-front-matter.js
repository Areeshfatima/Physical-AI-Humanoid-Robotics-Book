#!/usr/bin/env node

const fs = require('fs').promises;
const { scanDocumentationFiles } = require('./utils/file-scanner');
const { 
  hasValidFrontMatterAtStart,
  hasInvalidYamlSyntax,
  parseYamlFrontMatter
} = require('./utils/frontmatter');
const { hasRequiredFields } = require('./utils/validator');

/**
 * Validates that a file has correct front matter
 * @param {string} filePath - Path to the file to validate
 * @returns {Object} Validation result
 */
async function validateFileFrontMatter(filePath) {
  try {
    // Check if file has valid front matter at start
    if (!(await hasValidFrontMatterAtStart(filePath))) {
      return {
        filePath,
        valid: false,
        errors: ['Missing or incorrectly positioned front matter']
      };
    }
    
    // Check if file has invalid YAML syntax
    if (await hasInvalidYamlSyntax(filePath)) {
      return {
        filePath,
        valid: false,
        errors: ['Invalid YAML syntax in front matter']
      };
    }
    
    // Parse the front matter and check for required fields
    const frontMatter = await parseYamlFrontMatter(filePath);
    const fieldValidation = hasRequiredFields(frontMatter);
    
    if (!fieldValidation.valid) {
      return {
        filePath,
        valid: false,
        errors: [`Missing required fields: ${fieldValidation.missingFields.join(', ')}`]
      };
    }
    
    return {
      filePath,
      valid: true,
      errors: []
    };
  } catch (error) {
    return {
      filePath,
      valid: false,
      errors: [`Error validating file: ${error.message}`]
    };
  }
}

/**
 * Validates all documentation files
 * @param {string} docsDir - Directory containing documentation files (default: 'my-book/docs/')
 */
async function validateAllFrontMatter(docsDir = 'my-book/docs/') {
  console.log(`Validating documentation files in ${docsDir}...`);
  
  // Get all documentation files
  const files = await scanDocumentationFiles(docsDir);
  console.log(`Found ${files.length} documentation files to validate`);
  
  const results = {
    total: files.length,
    valid: 0,
    invalid: 0,
    validationResults: []
  };
  
  // Validate each file
  for (const file of files) {
    console.log(`Validating: ${file}`);
    
    const result = await validateFileFrontMatter(file);
    results.validationResults.push(result);
    
    if (result.valid) {
      results.valid++;
      console.log(`  - Status: VALID`);
    } else {
      results.invalid++;
      console.log(`  - Status: INVALID - ${result.errors.join('; ')}`);
    }
  }
  
  console.log('\n--- Validation Summary ---');
  console.log(`Total files validated: ${results.total}`);
  console.log(`Valid files: ${results.valid}`);
  console.log(`Invalid files: ${results.invalid}`);
  
  if (results.invalid > 0) {
    console.log('\nInvalid files:');
    for (const result of results.validationResults) {
      if (!result.valid) {
        console.log(`  - ${result.filePath}: ${result.errors.join(', ')}`);
      }
    }
  }
  
  return results;
}

// For testing purposes, run immediately if this file is executed directly
if (require.main === module) {
  const docsDir = process.argv[2] || 'my-book/docs/';
  validateAllFrontMatter(docsDir)
    .then(results => {
      console.log('\nValidation process completed.');
      process.exit(results.invalid > 0 ? 1 : 0);
    })
    .catch(error => {
      console.error('Validation process failed:', error);
      process.exit(1);
    });
}

module.exports = {
  validateFileFrontMatter,
  validateAllFrontMatter
};