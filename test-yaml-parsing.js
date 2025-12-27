const fs = require('fs').promises;
const yaml = require('js-yaml');

async function testYamlParsing(filePath) {
  try {
    console.log(`Testing YAML parsing for: ${filePath}`);
    const content = await fs.readFile(filePath, 'utf8');
    
    // Check if file starts with front matter
    if (!content.startsWith('---\n')) {
      console.log(`  - No front matter found (doesn't start with ---)`);
      return true;
    }
    
    // Extract the front matter (between first --- and the next ---)
    const lines = content.split('\n');
    let frontMatterEndIndex = -1;
    for (let i = 1; i < lines.length; i++) {
      if (lines[i].trim() === '---') {
        frontMatterEndIndex = i;
        break;
      }
    }
    
    if (frontMatterEndIndex === -1) {
      console.log(`  - No closing --- found for front matter`);
      return false;
    }
    
    const yamlContent = lines.slice(1, frontMatterEndIndex).join('\n');
    console.log(`  - YAML content:`);
    console.log(yamlContent);
    
    try {
      const parsed = yaml.load(yamlContent);
      console.log(`  - Parsed successfully:`, parsed);
      return true;
    } catch (yamlErr) {
      console.log(`  - YAML parsing error:`, yamlErr.message);
      return false;
    }
  } catch (error) {
    console.log(`  - File reading error:`, error.message);
    return false;
  }
}

// Test a few files
(async () => {
  const files = [
    'my-book/docs/intro.md',
    'my-book/docs/module-1-ros2/chapter-1.md',
    'my-book/docs/module-2-digital-twin/chapter-1.md'
  ];
  
  for (const file of files) {
    const result = await testYamlParsing(file);
    console.log(`File ${file}: ${result ? 'VALID' : 'INVALID'}\n`);
  }
})();