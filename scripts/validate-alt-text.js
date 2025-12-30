/**
 * Alt text validation for accessibility compliance
 * Scans markdown files to ensure all images have appropriate alt text
 */

const fs = require('fs');
const path = require('path');

// Define the project paths
const DOCS_DIR = path.join(__dirname, '..', 'docs');

// Function to find all markdown files in docs directory recursively
function findMarkdownFiles(dir = DOCS_DIR) {
  let results = [];
  const items = fs.readdirSync(dir);

  for (const item of items) {
    const itemPath = path.join(dir, item);
    const stat = fs.statSync(itemPath);

    if (stat.isDirectory()) {
      results = results.concat(findMarkdownFiles(itemPath));
    } else if (path.extname(item) === '.md') {
      results.push(itemPath);
    }
  }

  return results;
}

// Regex to match markdown image references
const IMG_REGEX = /!\[([^\]]*)\]\(([^)]+)\)/g;
// Regex to match HTML img tags
const HTML_IMG_REGEX = /<img[^>]+alt\s*=\s*["']([^"']*)["'][^>]*>/gi;

/**
 * Validates alt text for accessibility
 * @param {string} altText - The alt text to validate
 * @returns {object} - Validation result
 */
function validateAltText(altText) {
  // Check if alt text is empty or just whitespace
  if (!altText || altText.trim().length === 0) {
    return {
      valid: false,
      message: 'Alt text is empty or contains only whitespace'
    };
  }

  // Check if alt text is purely decorative (too short to be descriptive)
  if (altText.trim().length < 3) {
    return {
      valid: false,
      message: `Alt text is too short (${altText.length} chars). Minimum recommended length is 3 characters.`
    };
  }

  // Check if alt text is placeholder-like
  const placeholderPatterns = [
    /^image$/,
    /^img$/,
    /^photo$/,
    /^picture$/,
    /^diagram$/,
    /^chart$/,
    /^graph$/,
    /^illustration$/,
    /^graphic$/,
    /^visual$/,
    /^caption$/,
    /^description$/,
    /^example$/,
    /^placeholder$/,
    /\bimage\b.*\b\d+\b/i,
    /\bimg\b.*\b\d+\b/i
  ];

  for (const pattern of placeholderPatterns) {
    if (pattern.test(altText.trim())) {
      return {
        valid: false,
        message: `Alt text appears to be a generic placeholder: "${altText.trim()}"`
      };
    }
  }

  return {
    valid: true,
    message: 'Alt text validation passed'
  };
}

/**
 * Scans markdown files for images and validates alt text
 * @returns {array} - Array of validation results
 */
function scanAltTexts() {
  const markdownFiles = findMarkdownFiles();

  const results = [];

  for (const file of markdownFiles) {
    const content = fs.readFileSync(file, 'utf8');

    // Check markdown-style images: ![alt text](path)
    let match;
    while ((match = IMG_REGEX.exec(content)) !== null) {
      const altText = match[1];
      const imagePath = match[2];

      const validationResult = validateAltText(altText);

      results.push({
        file: path.relative(DOCS_DIR, file),
        line: content.substring(0, match.index).split('\n').length,
        altText: altText,
        imagePath: imagePath,
        type: 'markdown',
        ...validationResult
      });
    }

    // Reset lastIndex for next regex
    IMG_REGEX.lastIndex = 0;

    // Check HTML img tags: <img alt="text" ... >
    while ((match = HTML_IMG_REGEX.exec(content)) !== null) {
      const altText = match[1];

      const validationResult = validateAltText(altText);

      results.push({
        file: path.relative(DOCS_DIR, file),
        line: content.substring(0, match.index).split('\n').length,
        altText: altText,
        type: 'html',
        ...validationResult
      });
    }

    // Reset lastIndex for next regex
    HTML_IMG_REGEX.lastIndex = 0;
  }

  return results;
}

/**
 * Generates accessibility recommendations
 * @param {object} validationResult - Result from validateAltText
 * @returns {string} - Recommendations
 */
function generateRecommendations(validationResult) {
  if (!validationResult.valid) {
    if (validationResult.message.includes('empty')) {
      return 'Add descriptive alt text that explains the image content and function';
    } else if (validationResult.message.includes('too short')) {
      return 'Expand alt text to be more descriptive of the image content';
    } else if (validationResult.message.includes('placeholder')) {
      return 'Replace generic placeholder text with specific, descriptive alt text';
    }
  }
  return 'Alt text is appropriate';
}

module.exports = {
  validateAltText,
  scanAltTexts,
  generateRecommendations
};

// If running as main script, validate all markdown files
if (require.main === module) {
  console.log('Scanning markdown files for alt text accessibility validation...\n');
  
  const results = scanAltTexts();
  
  let validCount = 0;
  let invalidCount = 0;
  
  for (const result of results) {
    console.log(`${result.valid ? '✓' : '✗'} File: ${result.file} (line: ${result.line})`);
    console.log(`  Type: ${result.type} - ${result.imagePath || 'HTML img tag'}`);
    console.log(`  Alt text: "${result.altText}"`);
    console.log(`  Status: ${result.message}`);
    console.log(`  Recommendations: ${generateRecommendations(result)}`);
    console.log('');
    
    if (result.valid) {
      validCount++;
    } else {
      invalidCount++;
    }
  }
  
  console.log(`\nSummary: ${validCount} valid alt texts, ${invalidCount} invalid alt texts`);
  
  if (invalidCount > 0) {
    console.log('\nAccessibility issues found! Please update alt texts as recommended.');
    process.exit(1); // Exit with error code if accessibility issues found
  } else {
    console.log('\nAll images have proper alt text for accessibility!');
  }
}