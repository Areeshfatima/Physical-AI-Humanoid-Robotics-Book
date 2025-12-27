const fs = require('fs');
const path = require('path');

// Function to generate a basic SVG placeholder with text
function createPlaceholder(text) {
  return `<svg width="400" height="200" xmlns="http://www.w3.org/2000/svg">
  <rect width="100%" height="100%" fill="#f0f0f0"/>
  <rect x="10" y="10" width="380" height="180" fill="#d3d3d3" stroke="#a9a9a9" stroke-width="2"/>
  <text x="200" y="90" font-family="Arial" font-size="16" text-anchor="middle" fill="#000000">${text}</text>
  <text x="200" y="115" font-family="Arial" font-size="12" text-anchor="middle" fill="#000000">Image placeholder</text>
  <path d="M50,50 L350,50 L350,150 L50,150 Z" fill="none" stroke="#a9a9a9" stroke-width="1"/>
  <path d="M50,50 L350,150 M350,50 L50,150" stroke="#a9a9a9" stroke-width="1"/>
</svg>`;
}

// Create placeholder images for each module and chapter
const modules = [
  '001-docusaurus-textbook',
  '003-digital-twin-gazebo-unity', 
  '004-isaac-sim-vslam-nav',
  '005-vla-systems'
];

modules.forEach(module => {
  for (let chapter = 1; chapter <= 4; chapter++) {
    const dir = `/mnt/e/Hackathon-1/physical-ai-humanoid-robotics/my-book/static/img/${module}/chapter-${chapter}`;
    if (!fs.existsSync(dir)) {
      fs.mkdirSync(dir, { recursive: true });
    }
    
    const placeholderPath = path.join(dir, 'placeholder.svg');
    const content = createPlaceholder(`${module} - Chapter ${chapter} Placeholder`);
    fs.writeFileSync(placeholderPath, content);
    
    // Also create a generic placeholder in the module directory
    const modulePlaceholderPath = path.join(`/mnt/e/Hackathon-1/physical-ai-humanoid-robotics/my-book/static/img/${module}`, 'module-placeholder.svg');
    const moduleContent = createPlaceholder(`${module} Module Placeholder`);
    fs.writeFileSync(modulePlaceholderPath, moduleContent);
  }
});

console.log('Placeholder images created successfully!');