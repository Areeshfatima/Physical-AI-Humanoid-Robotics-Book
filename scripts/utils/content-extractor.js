/**
 * Extract content from markdown file, separating front matter from body
 * @param {string} content - Full content of the markdown file
 * @returns {Object} Object with frontMatter and bodyContent
 */
function extractContent(content) {
  // Check if content starts with front matter delimiter
  if (!content.startsWith('---\n')) {
    // No front matter, entire content is body
    return {
      frontMatter: null,
      bodyContent: content
    };
  }

  // Split into lines to find front matter boundaries
  const lines = content.split('\n');
  
  // Find the end of front matter (first line after --- that is also ---)
  let frontMatterEndIndex = -1;
  for (let i = 1; i < lines.length; i++) {
    if (lines[i].trim() === '---') {
      frontMatterEndIndex = i;
      break;
    }
  }

  if (frontMatterEndIndex === -1) {
    // No closing --- found, so treat as no front matter
    return {
      frontMatter: null,
      bodyContent: content
    };
  }

  // Extract front matter and body content
  const frontMatter = lines.slice(1, frontMatterEndIndex).join('\n');
  // Include empty line after front matter in body content if exists
  let bodyContent = '';
  if (frontMatterEndIndex + 1 < lines.length) {
    bodyContent = lines.slice(frontMatterEndIndex + 1).join('\n');
  }

  return {
    frontMatter,
    bodyContent
  };
}

/**
 * Reconstruct content from front matter and body
 * @param {string|null} frontMatter - Front matter content (without --- delimiters)
 * @param {string} bodyContent - Body content of the file
 * @returns {string} Reconstructed file content
 */
function reconstructContent(frontMatter, bodyContent) {
  if (!frontMatter) {
    return bodyContent;
  }
  
  return `---\n${frontMatter}\n---\n${bodyContent}`;
}

module.exports = {
  extractContent,
  reconstructContent
};