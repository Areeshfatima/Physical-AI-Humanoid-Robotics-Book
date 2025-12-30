/**
 * Image validation framework to enforce 5MB size limit
 * Validates image size and format for Docusaurus textbook
 */

const fs = require('fs');
const path = require('path');

// Maximum file size in bytes (5MB)
const MAX_FILE_SIZE = 5 * 1024 * 1024;

// Supported image formats
const SUPPORTED_FORMATS = ['.png', '.jpg', '.jpeg', '.gif', '.svg'];

/**
 * Sanitizes an image file, particularly for security
 * @param {string} imagePath - Path to the image file
 * @returns {object} - Sanitization result with success status and message
 */
function sanitizeImage(imagePath) {
  try {
    const ext = path.extname(imagePath).toLowerCase();

    // For SVG files, perform additional sanitization beyond basic validation
    if (ext === '.svg') {
      let content = fs.readFileSync(imagePath, 'utf8');

      // Remove potentially dangerous content from SVG
      const originalContent = content;

      // Remove script tags completely
      content = content.replace(/<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi, '');

      // Remove event handlers
      content = content.replace(/\s+(on\w+)\s*=\s*["']?[^"'\s>]*["'\s>]/gi, '');

      // Remove javascript: URLs
      content = content.replace(/javascript:[^"'\s<]*/gi, '');

      // Remove data: URLs that could contain scripts (more conservative approach)
      content = content.replace(/data:\s*[^,\s]+\s*,\s*(?:[^",\s]+,?)*\s*["'\s>]/gi, '');

      // If content was modified, write back sanitized version
      if (content !== originalContent) {
        fs.writeFileSync(imagePath, content);
        return {
          success: true,
          message: 'SVG sanitized successfully, potentially dangerous content removed',
          sanitized: true
        };
      }
    }

    return {
      success: true,
      message: 'No sanitization needed or image sanitized successfully',
      sanitized: false
    };

  } catch (error) {
    return {
      success: false,
      message: `Error sanitizing image: ${error.message}`
    };
  }
}

/**
 * Validates an image file
 * @param {string} imagePath - Path to the image file
 * @returns {object} - Validation result with success status and message
 */
function validateImage(imagePath) {
  try {
    // Check if file exists
    if (!fs.existsSync(imagePath)) {
      return {
        success: false,
        message: `Image file does not exist: ${imagePath}`
      };
    }

    // Check file size
    const stats = fs.statSync(imagePath);
    if (stats.size > MAX_FILE_SIZE) {
      return {
        success: false,
        message: `Image exceeds size limit: ${formatBytes(stats.size)} > ${formatBytes(MAX_FILE_SIZE)}`,
        currentSize: stats.size,
        maxSize: MAX_FILE_SIZE
      };
    }

    // Check file format
    const ext = path.extname(imagePath).toLowerCase();
    if (!SUPPORTED_FORMATS.includes(ext)) {
      return {
        success: false,
        message: `Unsupported image format: ${ext}. Supported formats: ${SUPPORTED_FORMATS.join(', ')}`
      };
    }

    // Additional validation for SVG files (security check)
    if (ext === '.svg') {
      const content = fs.readFileSync(imagePath, 'utf8');

      // Check for potentially dangerous content in SVG
      const dangerousPatterns = [
        /<script/i,
        /javascript:/i,
        /onload=/i,
        /onclick=/i,
        /onerror=/i,
        /<iframe/i,
        /<object/i,
        /<embed/i
      ];

      for (const pattern of dangerousPatterns) {
        if (pattern.test(content)) {
          return {
            success: false,
            message: `SVG contains potentially dangerous content: ${pattern.toString()}`
          };
        }
      }
    }

    return {
      success: true,
      message: 'Image validation passed',
      fileSize: stats.size,
      format: ext
    };
  } catch (error) {
    return {
      success: false,
      message: `Error validating image: ${error.message}`
    };
  }
}

/**
 * Validates all images in a directory
 * @param {string} directory - Directory to scan for images
 * @returns {array} - Array of validation results
 */
function validateImageDirectory(directory) {
  const results = [];
  
  if (!fs.existsSync(directory)) {
    return [{ 
      success: false, 
      message: `Directory does not exist: ${directory}` 
    }];
  }

  const files = fs.readdirSync(directory);
  
  for (const file of files) {
    const filePath = path.join(directory, file);
    const stat = fs.statSync(filePath);
    
    if (stat.isDirectory()) {
      // Recursively validate subdirectories
      const subResults = validateImageDirectory(filePath);
      results.push(...subResults);
    } else {
      const ext = path.extname(file).toLowerCase();
      if (SUPPORTED_FORMATS.includes(ext)) {
        const result = validateImage(filePath);
        result.filePath = filePath;
        results.push(result);
      }
    }
  }
  
  return results;
}

/**
 * Formats bytes to human-readable format
 * @param {number} bytes - Number of bytes
 * @returns {string} - Formatted size
 */
function formatBytes(bytes) {
  if (bytes === 0) return '0 Bytes';
  const k = 1024;
  const sizes = ['Bytes', 'KB', 'MB', 'GB'];
  const i = Math.floor(Math.log(bytes) / Math.log(k));
  return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
}

/**
 * Generates optimization recommendations
 * @param {object} validationResult - Result from validateImage
 * @returns {string} - Optimization recommendations
 */
function getOptimizationRecommendations(validationResult) {
  if (!validationResult.success && validationResult.message.includes('exceeds size limit')) {
    return [
      'Compress the image using tools like TinyPNG or ImageOptim',
      'Consider using SVG format for diagrams and illustrations',
      'Use appropriate resolution for web display (72-96 DPI)',
      'For photographs, use JPEG with 80-85% quality settings'
    ].join('\n- ');
  }
  return 'No optimization needed';
}

module.exports = {
  validateImage,
  validateImageDirectory,
  sanitizeImage,
  formatBytes,
  getOptimizationRecommendations,
  MAX_FILE_SIZE,
  SUPPORTED_FORMATS
};

// If running as main script, validate a directory
if (require.main === module) {
  const directory = process.argv[2] || './static/img';
  console.log(`Validating images in directory: ${directory}\n`);
  
  const results = validateImageDirectory(directory);
  
  let validCount = 0;
  let invalidCount = 0;
  
  for (const result of results) {
    if (result.filePath) {
      console.log(`${result.success ? '✓' : '✗'} ${result.filePath}`);
      if (!result.success) {
        console.log(`  Error: ${result.message}`);
        if (result.currentSize) {
          console.log(`  Size: ${formatBytes(result.currentSize)} (Max: ${formatBytes(MAX_FILE_SIZE)})`);
        }
        console.log(`  Recommendations: ${getOptimizationRecommendations(result)}\n`);
      } else {
        console.log(`  Size: ${formatBytes(result.fileSize)}, Format: ${result.format}\n`);
        validCount++;
      }
    }
  }
  
  console.log(`\nSummary: ${validCount} valid, ${invalidCount} invalid`);
}