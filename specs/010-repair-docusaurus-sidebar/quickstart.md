# Quickstart: Repair Docusaurus Sidebar and Document IDs

## Overview
This tool helps repair invalid, duplicate, or missing document IDs and synchronize the sidebar.ts file with the actual docs/ directory structure. It scans your documentation files, identifies ID issues, and repairs them while preserving content and regenerating the sidebar.

## Prerequisites
- Node.js 18+ installed on your system
- Access to the documentation files in `my-book/docs/` directory
- Docusaurus v3.9.2 project setup
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
1. Run the repair tool to fix document IDs and regenerate sidebar:
   ```bash
   node scripts/repair-sidebar-ids.js
   ```

2. Run in dry-run mode to preview changes without applying them:
   ```bash
   node scripts/repair-sidebar-ids.js --dry-run
   ```

3. Validate that the fixes worked:
   ```bash
   node scripts/validate-sidebar-ids.js
   ```

## Test Scenarios
1. **With backup**: Run with `--backup` flag to preserve original files before modification
   ```bash
   node scripts/repair-sidebar-ids.js --backup
   ```

2. **Performance test**: Process a large number of files to verify performance requirements
   ```bash
   node scripts/repair-sidebar-ids.js --performance-test
   ```

3. **Directory structure test**: Process deeply nested directory structures
   ```bash
   node scripts/repair-sidebar-ids.js --test-nested-dirs
   ```

## Verification
After running the tool:
1. Verify that all markdown files have unique, valid, ASCII-only IDs
2. Run the Docusaurus build to ensure no ID-related errors:
   ```bash
   cd my-book && npm run build
   ```
3. Start the development server to verify functionality:
   ```bash
   npm start
   ```
4. Check that sidebar.ts has been regenerated to match the docs tree structure
5. Verify that intro.md exists and is the first entry in the sidebar

## Expected Output
- Files with invalid or missing IDs will have appropriate unique IDs assigned
- All document titles will be valid and non-empty
- All documents will have numeric sidebar_position values
- The sidebar.ts file will be regenerated to match the actual docs structure
- Deeply nested directory structures will create corresponding sidebar categories
- All document content will remain unchanged
- Processed files will have properly formatted Docusaurus-compatible front matter
- The tool will continue processing after encountering errors in individual files