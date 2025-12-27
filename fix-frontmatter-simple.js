#!/usr/bin/env node

// Simple script to fix remaining front matter issues in documentation files
const fs = require('fs').promises;
const path = require('path');
const { glob } = require('glob');

async function fixFrontMatterIssues() {
  console.log('Starting front matter fix process...');
  
  // Find all markdown files in docs directory
  const files = glob.sync('my-book/docs/**/*.md', { absolute: true });
  
  // Filter out hidden files (starting with underscore)
  const nonHiddenFiles = files.filter(file => {
    const basename = path.basename(file);
    return !basename.startsWith('_');
  });

  console.log(`Found ${nonHiddenFiles.length} documentation files to process...`);

  for (const file of nonHiddenFiles) {
    try {
      let content = await fs.readFile(file, 'utf8');
      
      // Check if file starts with front matter
      if (content.startsWith('---\n')) {
        // Extract the front matter (content between the first --- and the next ---)
        const lines = content.split('\n');
        let frontMatterEndIndex = -1;
        for (let i = 1; i < lines.length; i++) {
          if (lines[i].trim() === '---') {
            frontMatterEndIndex = i;
            break;
          }
        }

        if (frontMatterEndIndex !== -1) {
          // Extract the YAML content
          const yamlLines = lines.slice(1, frontMatterEndIndex);
          let updatedYamlLines = [];
          let hasChanges = false;

          for (const line of yamlLines) {
            // Fix empty keywords field
            if (line.trim().startsWith('keywords:') && line.trim() === 'keywords:') {
              // Look ahead to see if the next line is part of the keywords array
              const currentLineIndex = yamlLines.indexOf(line);
              if (currentLineIndex + 1 < yamlLines.length) {
                const nextLine = yamlLines[currentLineIndex + 1];
                if (nextLine.trim().startsWith('- ')) {
                  // This is part of a proper array, keep the current line
                  updatedYamlLines.push(line);
                } else {
                  // Fix empty keywords by setting it to an empty array
                  updatedYamlLines.push('keywords: []');
                  hasChanges = true;
                }
              } else {
                // End of front matter, fix empty keywords
                updatedYamlLines.push('keywords: []');
                hasChanges = true;
              }
            } else {
              updatedYamlLines.push(line);
            }
          }

          if (hasChanges) {
            // Reconstruct content with fixed front matter
            const newContent = [
              '---',
              ...updatedYamlLines,
              '---',
              ...lines.slice(frontMatterEndIndex + 1) // Include everything after the closing ---
            ].join('\n');

            await fs.writeFile(file, newContent);
            console.log(`✓ Fixed front matter in: ${file}`);
          }
        }
      }
    } catch (error) {
      console.error(`✗ Error processing file ${file}:`, error);
    }
  }
  
  console.log('Front matter fixing completed!');
}

// Run the fix function
fixFrontMatterIssues().catch(console.error);