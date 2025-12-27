/**
 * Validates that a string is valid YAML
 * @param {string} yamlString - The YAML string to validate
 * @returns {boolean} True if the string is valid YAML
 */
function isValidYaml(yamlString) {
  try {
    return require('js-yaml').load(yamlString) !== undefined;
  } catch (e) {
    return false;
  }
}

/**
 * Checks if front matter has the required fields (id, title, sidebar_position)
 * @param {object} frontMatter - The front matter object to check
 * @returns {object} Result with valid flag and list of missing fields
 */
function hasRequiredFields(frontMatter) {
  if (!frontMatter) {
    return {
      valid: false,
      missingFields: ['id', 'title', 'sidebar_position']
    };
  }

  const missingFields = [];
  if (!frontMatter.id) {
    missingFields.push('id');
  }
  if (!frontMatter.title) {
    missingFields.push('title');
  }
  if (!frontMatter.sidebar_position && frontMatter.sidebar_position !== 0) {
    missingFields.push('sidebar_position');
  }

  return {
    valid: missingFields.length === 0,
    missingFields
  };
}

/**
 * Validates that an ID is properly formatted
 * @param {string} id - The ID to validate
 * @returns {boolean} True if ID is valid
 */
function isValidId(id) {
  if (typeof id !== 'string') {
    return false;
  }

  // Check that ID only contains allowed characters (alphanumeric, hyphens, underscores)
  // and doesn't start/end with special characters that have meaning in YAML
  return /^[a-zA-Z0-9][a-zA-Z0-9_-]*[a-zA-Z0-9]$|^[a-zA-Z0-9]$/.test(id);
}

/**
 * Validates that a title is properly formatted
 * @param {string} title - The title to validate
 * @returns {boolean} True if title is valid
 */
function isValidTitle(title) {
  if (typeof title !== 'string') {
    return false;
  }

  // Title should be non-empty when trimmed
  return title.trim().length > 0;
}

/**
 * Validates that a sidebar position is numeric
 * @param {*} position - The position to validate
 * @returns {boolean} True if position is valid
 */
function isValidSidebarPosition(position) {
  return typeof position === 'number' || /^\d+$/.test(position);
}

/**
 * Validates that a title is properly formatted
 * @param {string} title - The title to validate
 * @returns {boolean} True if title is valid
 */
function isValidTitle(title) {
  if (typeof title !== 'string') {
    return false;
  }

  // Title should be non-empty when trimmed
  return title.trim().length > 0;
}


/**
 * Validates an entire front matter object
 * @param {object} frontMatter - The front matter object to validate
 * @param {Set} existingIds - Optional set of existing IDs to check uniqueness against
 * @returns {object} Validation result
 */
function validateFrontMatter(frontMatter, existingIds = new Set()) {
  if (!frontMatter || typeof frontMatter !== 'object') {
    return {
      valid: false,
      errors: ['Front matter is not a valid object']
    };
  }

  const requiredFieldsResult = hasRequiredFields(frontMatter);
  const errors = [];
  
  if (!requiredFieldsResult.valid) {
    errors.push(`Missing required fields: ${requiredFieldsResult.missingFields.join(', ')}`);
  }
  
  if (frontMatter.id && !isValidId(frontMatter.id)) {
    errors.push('ID contains invalid characters or format');
  }
  
  if (frontMatter.id && existingIds.size > 0 && existingIds.has(frontMatter.id)) {
    errors.push('ID is not unique across the documentation set');
  }

  if (frontMatter.title && !isValidTitle(frontMatter.title)) {
    errors.push('Title is invalid (empty or only whitespace)');
  }

  if (frontMatter.sidebar_position !== undefined && !isValidSidebarPosition(frontMatter.sidebar_position)) {
    errors.push('Sidebar position is not numeric');
  }

  return {
    valid: errors.length === 0,
    errors
  };
}

module.exports = {
  isValidYaml,
  hasRequiredFields,
  isValidId,
  isValidTitle,
  isValidSidebarPosition,
  validateFrontMatter
};