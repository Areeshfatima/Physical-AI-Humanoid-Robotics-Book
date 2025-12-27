# Quickstart: Markdown Front Matter Fix

## Overview
This tool helps fix invalid or missing YAML front matter in Docusaurus documentation files. It scans your documentation directory, identifies files with front matter issues, and repairs them while preserving the content.

## Prerequisites
- Node.js 18+ installed on your system
- Access to the documentation files in `my-book/docs/` directory
- (Optional) npm or yarn for dependency management

## Setup
1. Navigate to the project root directory:
   ```bash
   cd /mnt/e/Hackathon-1/physical-ai-humanoid-robotics
   ```

2. Install dependencies (if not already installed):
   ```bash
   npm install js-yaml glob
   ```

## Usage
1. Run the front matter fix tool:
   ```bash
   node scripts/fix-front-matter.js
   ```

2. (Optional) Validate that the fixes worked:
   ```bash
   node scripts/validate-front-matter.js
   ```

## Test Scenarios
1. **With backup**: Run with `--backup` flag to preserve original files before modification
   ```bash
   node scripts/fix-front-matter.js --backup
   ```

2. **Dry run**: Run with `--dry-run` flag to see what changes would be made without applying them
   ```bash
   node scripts/fix-front-matter.js --dry-run
   ```

3. **Specific directory**: Process only a specific subdirectory
   ```bash
   node scripts/fix-front-matter.js --dir my-book/docs/intro
   ```

## Verification
After running the tool:
1. Verify that all markdown files have valid YAML front matter
2. Run the Docusaurus build to ensure no front matter parsing errors:
   ```bash
   cd my-book && npm run build
   ```
3. Start the development server to verify functionality:
   ```bash
   npm start
   ```

## Expected Output
- Files with missing front matter will have default front matter added
- Files with invalid front matter will have the invalid YAML fixed
- All document content will remain unchanged
- Processed files will have valid Docusaurus-compatible front matter