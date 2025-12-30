const fs = require('fs');
const path = require('path');

/**
 * Utility functions for backing up files before modification
 */

/**
 * Create a backup of a file
 * @param {string} filePath - Path to the file to backup
 * @returns {string} - Path to the backup file
 */
function createBackup(filePath) {
  const backupDir = path.join(path.dirname(filePath), 'backups');
  
  // Create backups directory if it doesn't exist
  if (!fs.existsSync(backupDir)) {
    fs.mkdirSync(backupDir, { recursive: true });
  }
  
  const fileName = path.basename(filePath);
  const backupPath = path.join(backupDir, `${fileName}.backup`);
  
  // Copy file to backup location
  fs.copyFileSync(filePath, backupPath);
  
  return backupPath;
}

/**
 * Restore a file from its backup
 * @param {string} filePath - Path to the original file
 * @returns {boolean} - True if restore was successful, false otherwise
 */
function restoreFromBackup(filePath) {
  const backupDir = path.join(path.dirname(filePath), 'backups');
  const fileName = path.basename(filePath);
  const backupPath = path.join(backupDir, `${fileName}.backup`);
  
  if (fs.existsSync(backupPath)) {
    fs.copyFileSync(backupPath, filePath);
    return true;
  }
  
  return false;
}

module.exports = {
  createBackup,
  restoreFromBackup
};