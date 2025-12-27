const fs = require('fs').promises;
const path = require('path');
const { scanDocumentationFiles } = require('./file-scanner');

/**
 * Generates a sidebar configuration from a directory structure
 * @param {string} docsDir - Documentation directory path
 * @returns {Promise<Object>} Sidebar configuration object
 */
async function generateComplexSidebarFromTree(docsDir = 'my-book/docs/') {
  const files = await scanDocumentationFiles(docsDir);

  // Create a hierarchical structure based on directory paths
  const sidebarStructure = {};

  for (const file of files) {
    const relativePath = path.relative(docsDir, file);
    const segments = relativePath.split(path.sep);

    let current = sidebarStructure;

    // Process each segment except the last one (the file name)
    for (let i = 0; i < segments.length - 1; i++) {
      const segment = segments[i];

      if (!current[segment]) {
        current[segment] = {
          type: 'category',
          label: formatLabel(segment),
          items: {}
        };
      }

      current = current[segment].items;
    }

    // Handle the file itself
    const fileName = segments[segments.length - 1];
    const idWithoutExt = fileName.replace(/\.(md|mdx)$/, '');
    const fullId = segments.slice(0, -1).concat([idWithoutExt]).join('/');

    current[idWithoutExt] = {
      type: 'doc',
      id: fullId
    };
  }

  return convertToSidebarFormat(sidebarStructure);
}

/**
 * Converts the nested structure to proper Docusaurus sidebar format
 */
function convertToSidebarFormat(structure) {
  const result = {
    textbookSidebar: [] // Use the textbookSidebar name to match the docusaurus.config.ts
  };

  // Process all items in the structure
  flattenStructure('', structure, result.textbookSidebar);

  return result;
}

/**
 * Flattens the nested structure into the appropriate format
 */
function flattenStructure(prefix, structure, items) {
  for (const [key, value] of Object.entries(structure)) {
    const prefixedKey = prefix ? `${prefix}/${key}` : key;
    
    if (value.type === 'category') {
      const category = {
        type: 'category',
        label: value.label,
        items: []
      };
      
      if (value.items && Object.keys(value.items).length > 0) {
        flattenStructure(prefixedKey, value.items, category.items);
      }
      
      items.push(category);
    } else if (value.type === 'doc') {
      items.push({
        type: 'doc',
        id: value.id
      });
    }
  }
}

/**
 * Formats a path segment into a readable label
 */
function formatLabel(segment) {
  // Replace hyphens and underscores with spaces and capitalize
  return segment
    .replace(/[-_]+/g, ' ')
    .replace(/\b\w/g, l => l.toUpperCase());
}

/**
 * Writes the sidebar configuration to a file
 * @param {Object} sidebarConfig - The sidebar configuration to write
 * @param {string} outputPath - Path to output the sidebar configuration
 * @returns {Promise<void>}
 */
async function writeSidebarToFile(sidebarConfig, outputPath = 'my-book/sidebars.js') {
  // Ensure the output directory exists
  await fs.mkdir(path.dirname(outputPath), { recursive: true });

  let outputContent;

  // Check if the output path indicates a TypeScript file
  if (outputPath.endsWith('.ts')) {
    // Format the configuration as a TypeScript module export
    outputContent = `import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = ${JSON.stringify(sidebarConfig, null, 2)};

export default sidebars;
`;
  } else {
    // Format the configuration as a JavaScript module export
    outputContent = `module.exports = ${JSON.stringify(sidebarConfig, null, 2)};\n`;
  }

  // Write the file
  await fs.writeFile(outputPath, outputContent);
}

/**
 * Generates a simple sidebar configuration with all documents listed directly
 */
async function generateSimpleSidebar(docsDir = 'my-book/docs/') {
  const files = await scanDocumentationFiles(docsDir);
  const items = [];

  for (const file of files) {
    const relativePath = path.relative(docsDir, file);
    const id = relativePath.replace(/\.(md|mdx)$/, '');

    items.push({
      type: 'doc',
      id: id
    });
  }

  return {
    docs: {
      type: 'category',
      label: 'Docs',
      items: items
    }
  };
}

module.exports = {
  generateComplexSidebarFromTree,
  writeSidebarToFile,
  generateSimpleSidebar
};