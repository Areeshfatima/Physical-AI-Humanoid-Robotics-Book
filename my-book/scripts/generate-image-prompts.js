/**
 * System for generating AI image prompts per chapter
 * Scans chapters for content that could benefit from visual representation
 */

const fs = require('fs');
const path = require('path');

// Define the project paths
const DOCS_DIR = path.join(__dirname, '..', 'docs');
const OUTPUT_FILE = path.join(__dirname, 'resources', '_image-prompts.md');

// Function to find all markdown files in docs directory recursively
function findMarkdownFiles(dir = DOCS_DIR) {
  let results = [];
  const items = fs.readdirSync(dir);

  for (const item of items) {
    const itemPath = path.join(dir, item);
    const stat = fs.statSync(itemPath);

    if (stat.isDirectory()) {
      results = results.concat(findMarkdownFiles(itemPath));
    } else if (path.extname(item) === '.md') {
      results.push(itemPath);
    }
  }

  return results;
}

/**
 * Generates an AI image prompt based on content
 * @param {string} content - The content to analyze
 * @param {string} context - The context (e.g., chapter title, section)
 * @returns {string} - Generated image prompt
 */
function generateImagePrompt(content, context) {
  // Extract key concepts from content
  const keyConcepts = extractKeyConcepts(content);
  
  if (keyConcepts.length === 0) {
    return null; // No suitable content for image generation
  }
  
  // Create a specific prompt based on detected concepts
  const conceptStr = keyConcepts.slice(0, 3).join(', '); // Use first 3 concepts
  
  // Determine image type based on content
  let imageType = 'diagram';
  if (content.toLowerCase().includes('architecture') || content.toLowerCase().includes('structure')) {
    imageType = 'architecture diagram';
  } else if (content.toLowerCase().includes('flow') || content.toLowerCase().includes('process')) {
    imageType = 'flowchart or process diagram';
  } else if (content.toLowerCase().includes('algorithm') || content.toLowerCase().includes('code')) {
    imageType = 'technical illustration';
  } else if (content.toLowerCase().includes('simulation') || content.toLowerCase().includes('robot')) {
    imageType = 'simulation diagram or robot illustration';
  }
  
  return `Create a clear, technical ${imageType} illustrating: ${conceptStr}. Style: clean, minimalist, educational. Colors: professional palette suitable for textbook. Detail: enough to be informative but not cluttered.`;
}

/**
 * Extracts key concepts from content that could be visualized
 * @param {string} content - Text content to analyze
 * @returns {array} - Array of key concepts
 */
function extractKeyConcepts(content) {
  // Simple keyword extraction - could be enhanced with NLP
  const keywords = [
    // Technical concepts
    'architecture', 'diagram', 'structure', 'system', 'component', 'module',
    'process', 'flow', 'workflow', 'algorithm', 'implementation',
    'framework', 'pipeline', 'network', 'protocol', 'interface',
    // Robotics/AI concepts
    'robot', 'simulation', 'sensor', 'actuator', 'control', 'feedback',
    'navigation', 'localization', 'mapping', 'SLAM', 'VSLAM',
    'neural network', 'AI', 'machine learning', 'model', 'inference',
    'perception', 'cognition', 'digital twin',
    // General concepts
    'data', 'information', 'communication', 'interaction', 'connection',
    'relationship', 'hierarchy', 'sequence', 'timeline', 'cycle'
  ];
  
  const foundConcepts = [];
  const contentLower = content.toLowerCase();
  
  for (const keyword of keywords) {
    if (contentLower.includes(keyword.toLowerCase()) && !foundConcepts.includes(keyword)) {
      foundConcepts.push(keyword);
    }
  }
  
  return foundConcepts;
}

/**
 * Scans all documentation files and generates image prompts
 */
function generateAllImagePrompts() {
  const markdownFiles = findMarkdownFiles().filter(file =>
    !file.includes('_image-prompts.md') // Don't scan the output file
  );

  const prompts = [];

  for (const file of markdownFiles) {
    const content = fs.readFileSync(file, 'utf8');

    // Extract title from frontmatter or first heading
    const titleMatch = content.match(/^# (.+)$/m) || content.match(/title:\s*["']?([^"'\n]+)["']?/m);
    const title = titleMatch ? titleMatch[1] : path.basename(file, '.md');

    // Generate prompt based on content
    const prompt = generateImagePrompt(content, title);

    if (prompt) {
      prompts.push({
        file: path.relative(DOCS_DIR, file),
        title: title,
        content: content.substring(0, 300), // First 300 chars for context
        prompt: prompt
      });
    }
  }

  return prompts;
}

/**
 * Saves image prompts to markdown file
 * @param {array} prompts - Array of generated prompts
 */
function savePromptsToFile(prompts) {
  let content = `# AI Image Prompts for Textbook Visuals\n\n`;
  content += `Generated on: ${new Date().toISOString()}\n\n`;
  
  for (let i = 0; i < prompts.length; i++) {
    const prompt = prompts[i];
    content += `## ${i + 1}. ${prompt.title}\n\n`;
    content += `**File**: \`${prompt.file}\`\n\n`;
    content += `**Context**: ${prompt.content.replace(/\n/g, ' ').substring(0, 100)}...\n\n`;
    content += `**AI Prompt**: ${prompt.prompt}\n\n---\n\n`;
  }
  
  fs.writeFileSync(OUTPUT_FILE, content);
  console.log(`Image prompts saved to: ${OUTPUT_FILE}`);
  console.log(`Generated prompts for ${prompts.length} documents`);
}

module.exports = {
  generateImagePrompt,
  extractKeyConcepts,
  generateAllImagePrompts,
  savePromptsToFile
};

// If running as main script, generate and save prompts
if (require.main === module) {
  console.log('Generating AI image prompts for textbook content...\n');
  
  const prompts = generateAllImagePrompts();
  
  if (prompts.length > 0) {
    savePromptsToFile(prompts);
  } else {
    console.log('No suitable content found for image generation.');
  }
}