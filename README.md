# Docusaurus Sidebar and Document ID Repair Tool

This tool fixes invalid, duplicate, or missing document IDs and synchronizes the sidebar.ts file with the actual docs/ directory structure in Docusaurus projects.

## Purpose

The repair tool addresses common Docusaurus documentation issues:
- Invalid, duplicate, or missing document IDs
- Out of sync sidebar.ts file
- Version loader crashes on intro.md
- Missing required fields in front matter
- Invalid YAML syntax in front matter

## Features

- Scans documentation files recursively in the docs directory
- Repairs missing or invalid front matter (id, title, sidebar_position)
- Regenerates the sidebar.ts file to match the directory structure
- Validates ID uniqueness across the documentation set
- Preserves content while only modifying front matter
- Handles deeply nested directory structures
- Ignores hidden markdown files (starting with underscore)
- Provides dry-run mode to preview changes

## Usage

```bash
# Basic usage
node scripts/repair-sidebar-ids.js

# With custom docs directory
node scripts/repair-sidebar-ids.js --docs-dir "custom/docs/path/"

# With custom sidebar output
node scripts/repair-sidebar-ids.js --sidebar-output "custom/sidebars.js"

# Dry run mode (preview changes without applying)
node scripts/repair-sidebar-ids.js --dry-run

# Verbose output
node scripts/repair-sidebar-ids.js --verbose

# Disable backup creation
node scripts/repair-sidebar-ids.js --no-backup
```

## Files Modified

The tool processes all `.md` and `.mdx` files in the documentation directory and creates backups before modification.

## Backup Policy

By default, the tool creates backups of files before modifying them. Backups are saved in the `backups/front-matter-fix/` directory.

## Validation

After running the repair tool, you can validate the fixes by running the Docusaurus build:
```bash
cd my-book
npm run build
```

Or run the development server:
```bash
npm start
```

## Error Handling

The tool logs errors when encountered and continues processing other files rather than stopping execution.

## Requirements

- Node.js (v18 or higher)
- Required npm packages: js-yaml, glob, fs