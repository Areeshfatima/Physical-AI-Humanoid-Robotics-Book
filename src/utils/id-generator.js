/**
 * Generates a valid ID from a filename
 * @param {string} filename - The filename to generate an ID from
 * @returns {string} A valid, unique ID
 */
function generateIdFromFilename(filename) {
  const path = require('path');
  
  // Extract basename without extension
  const baseName = path.basename(filename, path.extname(filename));
  
  // Replace non-alphanumeric characters with hyphens
  const id = baseName
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, '-')
    .replace(/(^-|-$)/g, ''); // Remove leading/trailing hyphens
  
  return id || 'unnamed'; // Provide a fallback if the result is empty
}

/**
 * Generates a title from a filename (capitalizing words and replacing separators with spaces)
 * @param {string} filename - The filename to generate a title from
 * @returns {string} A formatted title
 */
function generateTitleFromFilename(filename) {
  const path = require('path');
  
  // Extract basename without extension
  const baseName = path.basename(filename, path.extname(filename));
  
  // Replace hyphens/underscores with spaces and capitalize first letter of each word
  const title = baseName
    .replace(/[-_]+/g, ' ')
    .replace(/\b\w/g, l => l.toUpperCase());
  
  return title || 'Untitled';
}

/**
 * Sanitizes an ID to ensure it follows proper format requirements
 * @param {string} id - The ID to sanitize
 * @returns {string} Sanitized ID
 */
function sanitizeId(id) {
  if (typeof id !== 'string') {
    id = String(id);
  }
  
  // Only allow alphanumeric characters, hyphens, and underscores
  let sanitizedId = id.replace(/[^a-zA-Z0-9_-]/g, '-');
  
  // Ensure it doesn't start or end with special characters that have meaning in YAML
  sanitizedId = sanitizedId.replace(/^-+|-+$/g, '');
  
  return sanitizedId || 'sanitized-id';
}

/**
 * Class to manage unique IDs and handle conflicts
 */
class IdManager {
  constructor() {
    this.usedIds = new Set();
  }
  
  /**
   * Add an ID to the manager, resolving conflicts if needed
   * @param {string} id - The ID to add
   * @returns {string} A unique ID after potential conflict resolution
   */
  addId(id) {
    if (!this.usedIds.has(id)) {
      this.usedIds.add(id);
      return id;
    }
    
    // If the ID already exists, find a unique variation
    let counter = 1;
    let uniqueId = `${id}-${counter}`;
    
    while (this.usedIds.has(uniqueId)) {
      counter++;
      uniqueId = `${id}-${counter}`;
    }
    
    this.usedIds.add(uniqueId);
    return uniqueId;
  }
  
  /**
   * Checks if an ID already exists
   * @param {string} id - The ID to check
   * @returns {boolean} True if ID exists
   */
  hasId(id) {
    return this.usedIds.has(id);
  }
  
  /**
   * Gets all IDs managed by this instance
   * @returns {string[]} Array of all managed IDs
   */
  getAllIds() {
    return Array.from(this.usedIds);
  }
}

module.exports = {
  generateIdFromFilename,
  generateTitleFromFilename,
  sanitizeId,
  IdManager
};