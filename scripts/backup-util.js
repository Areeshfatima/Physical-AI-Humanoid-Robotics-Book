const fs = require('fs').promises;
const path = require('path');

/**
 * Creates a backup of a file before modification
 * @param {string} filePath - Path to the file to backup
 * @param {string} backupDir - Directory to store backups (default: 'backups/')
 * @returns {Promise<string>} Path to the created backup file
 */
async function createBackup(filePath, backupDir = 'backups/front-matter-fix') {
  try {
    // Ensure backup directory exists
    await fs.mkdir(backupDir, { recursive: true });
    
    // Create backup file path
    const fileName = path.basename(filePath);
    const fileDir = path.dirname(filePath);
    const relativeDir = path.relative('.', fileDir);
    const backupPath = path.join(backupDir, relativeDir, fileName);
    
    // Ensure subdirectories exist in backup location
    await fs.mkdir(path.dirname(backupPath), { recursive: true });
    
    // Read the original file content
    const originalContent = await fs.readFile(filePath, 'utf8');
    
    // Write content to backup location
    await fs.writeFile(backupPath, originalContent);
    
    console.log(`Backup created: ${filePath} -> ${backupPath}`);
    return backupPath;
  } catch (error) {
    console.error(`Error creating backup for ${filePath}:`, error);
    throw error;
  }
}

module.exports = {
  createBackup
};