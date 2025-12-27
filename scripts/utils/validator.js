/**
 * Check if the front matter object has required fields (id and title)
 * @param {Object} frontMatter - The parsed front matter object
 * @returns {Object} Object with validation results
 */
function hasRequiredFields(frontMatter) {
  if (!frontMatter) {
    return {
      valid: false,
      missingFields: ['id', 'title']
    };
  }

  const missingFields = [];
  if (!frontMatter.id) {
    missingFields.push('id');
  }
  if (!frontMatter.title) {
    missingFields.push('title');
  }

  return {
    valid: missingFields.length === 0,
    missingFields
  };
}

/**
 * Add missing required fields to front matter
 * @param {Object} frontMatter - The parsed front matter object
 * @param {string} filename - The name of the file (to generate id and title if needed)
 * @returns {Object} Updated front matter with required fields
 */
function addRequiredFields(frontMatter, filename) {
  const updatedFrontMatter = { ...frontMatter };

  // Generate id if missing
  if (!updatedFrontMatter.id) {
    // Use filename without extension as default id
    const id = filename
      .replace(/^.*[\\/]/, '') // Get basename
      .replace(/\.[^/.]+$/, '') // Remove extension
      .toLowerCase()
      .replace(/[^a-z0-9]+/g, '-') // Replace non-alphanumeric with hyphens
      .replace(/(^-|-$)/g, ''); // Remove leading/trailing hyphens
    updatedFrontMatter.id = id;
  }

  // Generate title if missing
  if (!updatedFrontMatter.title) {
    // Use filename to create a title
    const title = filename
      .replace(/^.*[\\/]/, '') // Get basename
      .replace(/\.[^/.]+$/, '') // Remove extension
      .replace(/[-_]+/g, ' ') // Replace hyphens/underscores with spaces
      .replace(/\b\w/g, l => l.toUpperCase()); // Capitalize first letter of each word
    updatedFrontMatter.title = title;
  }

  return updatedFrontMatter;
}

/**
 * Validates the structure of front matter
 * @param {Object} frontMatter - The parsed front matter object
 * @returns {Object} Validation result with details
 */
function validateFrontMatterStructure(frontMatter) {
  if (!frontMatter || typeof frontMatter !== 'object') {
    return {
      valid: false,
      errors: ['Front matter is not a valid object']
    };
  }

  // Check for required fields
  const requiredFieldsResult = hasRequiredFields(frontMatter);
  
  return {
    valid: requiredFieldsResult.valid,
    errors: requiredFieldsResult.missingFields.length > 0 
      ? [`Missing required fields: ${requiredFieldsResult.missingFields.join(', ')}`] 
      : []
  };
}

module.exports = {
  hasRequiredFields,
  addRequiredFields,
  validateFrontMatterStructure
};