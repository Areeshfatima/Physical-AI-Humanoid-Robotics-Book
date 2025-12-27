#!/usr/bin/env node

// Comprehensive script to fix all remaining front matter issues
const fs = require('fs').promises;
const path = require('path');

async function fixEmptyKeywords() {
  console.log('Starting comprehensive front matter fix process...');
  
  // List of files with empty keywords that need fixing
  const filesToFix = [
    'my-book/docs/module-1-ros2/chapter-1.md',
    'my-book/docs/module-1-ros2/chapter-2.md', 
    'my-book/docs/module-1-ros2/chapter-3.md',
    'my-book/docs/module-1-ros2/chapter-4.md',
    'my-book/docs/module-1-ros2/index.md',
    'my-book/docs/module-2-digital-twin/chapter-1.md',
    'my-book/docs/module-2-digital-twin/chapter-2.md',
    'my-book/docs/module-2-digital-twin/chapter-3.md',
    'my-book/docs/module-2-digital-twin/chapter-4.md',
    'my-book/docs/module-2-digital-twin/index.md',
    'my-book/docs/module-3-isaac/chapter-1.md',
    'my-book/docs/module-3-isaac/chapter-2.md',
    'my-book/docs/module-3-isaac/chapter-3.md',
    'my-book/docs/module-3-isaac/chapter-4.md',
    'my-book/docs/module-3-isaac/index.md',
    'my-book/docs/module-4-vla/chapter-1.md',
    'my-book/docs/module-4-vla/chapter-2.md',
    'my-book/docs/module-4-vla/chapter-3.md',
    'my-book/docs/module-4-vla/chapter-4.md',
    'my-book/docs/module-4-vla/index.md',
  ];

  for (const filePath of filesToFix) {
    try {
      const fullPath = path.join(__dirname, filePath);
      let content = await fs.readFile(fullPath, 'utf8');
      
      // Look for empty keywords lines
      const lines = content.split('\n');
      const newLines = [];
      let i = 0;
      
      while (i < lines.length) {
        // Check if current line is 'keywords:' with nothing after it
        if (lines[i].trim() === 'keywords:' && 
            i + 1 < lines.length && 
            (lines[i+1].trim() === '' || lines[i+1].startsWith(' ') || lines[i+1].startsWith('---'))) {
          // Replace the empty keywords line with keywords: []
          newLines.push('keywords: []');
          newLines.push(lines[i+1]); // include the next line
          i += 2;
        } 
        // Check for cases where keywords is followed by more specific items
        else if (lines[i].trim() === 'keywords:' && 
                 i + 1 < lines.length && 
                 lines[i+1].trim().startsWith('- ')) {
          // This is a proper array format, just add both lines
          newLines.push(lines[i]);
          newLines.push(lines[i+1]);
          i += 2;
        }
        else {
          // Normal line, add as is
          newLines.push(lines[i]);
          i++;
        }
      }
      
      const newContent = newLines.join('\n');
      
      // Only write if content changed
      if (newContent !== content) {
        await fs.writeFile(fullPath, newContent);
        console.log(`✓ Fixed empty keywords in: ${filePath}`);
      } else {
        console.log(`- Already valid: ${filePath}`);
      }
    } catch (error) {
      console.error(`✗ Error processing file ${filePath}:`, error.message);
    }
  }
  
  console.log('Comprehensive front matter fixing completed!');
}

fixEmptyKeywords().catch(console.error);