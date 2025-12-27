const fs = require('fs');
const yaml = require('js-yaml');

// Read the file
const content = fs.readFileSync('/mnt/e/Hackathon-1/physical-ai-humanoid-robotics/my-book/docs/module-2-digital-twin/chapter-3.md', 'utf8');

// Extract the front matter
const lines = content.split('\n');
let frontMatterEndIndex = -1;
for (let i = 1; i < lines.length; i++) {
  if (lines[i].trim() === '---') {
    frontMatterEndIndex = i;
    break;
  }
}

if (frontMatterEndIndex === -1) {
  console.log('No closing --- found');
  return;
}

const yamlContent = lines.slice(1, frontMatterEndIndex).join('\n');

console.log('YAML content:');
console.log(yamlContent);
console.log('\n---\n');

// Parse the YAML
const parsed = yaml.load(yamlContent);
console.log('Parsed object:');
console.log(JSON.stringify(parsed, null, 2));

// Check the keywords
console.log('\nKeywords array:');
console.log(parsed.keywords);

// Check each keyword value specifically
parsed.keywords.forEach((kw, index) => {
  console.log(`Keyword ${index}: "${kw}"`);
  console.log(`  Type: ${typeof kw}`);
  console.log(`  Length: ${kw.length}`);
  console.log(`  Starts with quote: ${kw.startsWith('"')}`);
  console.log(`  Ends with quote: ${kw.endsWith('"')}`);
  console.log(`  Contains \\": ${kw.includes('\\"')}`);
  console.log(`  Character codes: ${Array.from(kw).map(c => c.charCodeAt(0))}`);
});