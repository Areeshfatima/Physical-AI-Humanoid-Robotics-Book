/**
 * Resolves conflicts when duplicate IDs are detected
 * @param {Set|string[]} existingIds - Set or array of existing IDs to check against
 * @param {string} newId - The new ID that might conflict with existing ones
 * @returns {string} A unique ID after resolving conflicts
 */
function resolveIdConflict(existingIds, newId) {
  // Convert to Set if it's an array for faster lookup
  const idSet = Array.isArray(existingIds) ? new Set(existingIds) : existingIds;
  
  if (!idSet.has(newId)) {
    return newId; // No conflict to resolve
  }

  // Use the pattern: originalId-1, originalId-2, etc.
  let suffix = 1;
  let uniqueId = `${newId}-${suffix}`;
  
  while (idSet.has(uniqueId)) {
    suffix++;
    uniqueId = `${newId}-${suffix}`;
  }
  
  return uniqueId;
}

/**
 * Class to manage ID conflicts and ensure uniqueness
 */
class ConflictResolver {
  constructor() {
    this.knownIds = new Set();
  }
  
  /**
   * Resolves ID conflicts by ensuring the ID is unique
   * @param {string} id - The ID to resolve conflicts for
   * @returns {string} A unique ID
   */
  resolveIdConflict(id) {
    if (!this.knownIds.has(id)) {
      this.knownIds.add(id);
      return id;
    }
    
    // If the ID already exists, find a unique variation
    let counter = 1;
    let uniqueId = `${id}-${counter}`;
    
    while (this.knownIds.has(uniqueId)) {
      counter++;
      uniqueId = `${id}-${counter}`;
    }
    
    this.knownIds.add(uniqueId);
    return uniqueId;
  }
  
  /**
   * Checks if an ID is unique among known IDs
   * @param {string} id - The ID to check
   * @returns {boolean} True if ID is unique
   */
  isIdUnique(id) {
    return !this.knownIds.has(id);
  }
  
  /**
   * Registers an ID as known (useful for tracking IDs from existing files)
   * @param {string} id - The ID to register
   */
  registerKnownId(id) {
    this.knownIds.add(id);
  }
  
  /**
   * Gets all known IDs
   * @returns {string[]} Array of all known IDs
   */
  getKnownIds() {
    return Array.from(this.knownIds);
  }
}

module.exports = {
  resolveIdConflict,
  ConflictResolver
};