# Command Line Interface Contract: Front Matter Fix Tool

## Overview
This document specifies the command-line interface for the markdown front matter fixing tool.

## Command Structure
```
node scripts/fix-front-matter.js [options]
```

## Options

### --help
Displays help information about the tool and its options.

**Type**: Flag
**Required**: No
**Default**: false

### --backup
Creates backup copies of files before modifying them.

**Type**: Flag
**Required**: No
**Default**: false
**Behavior**: When enabled, the tool creates .bak files before modifying any markdown files.

### --dry-run
Performs a dry run without modifying any files.

**Type**: Flag
**Required**: No
**Default**: false
**Behavior**: The tool reports what changes would be made without actually modifying files.

### --dir [path]
Specifies the directory to process (default is my-book/docs/).

**Type**: String with argument
**Required**: No
**Default**: "my-book/docs/"
**Validation**: Must be a valid directory path containing markdown files.

### --validate-only
Only validates front matter without making changes.

**Type**: Flag
**Required**: No
**Default**: false
**Behavior**: The tool checks all files for valid front matter and reports issues without fixing them.

## Exit Codes
- 0: Success, all operations completed without errors
- 1: General error occurred during processing
- 2: Invalid arguments provided
- 3: Failed to read or write files

## Input Format
The tool processes markdown files (.md and .mdx extensions) that may contain YAML front matter.

### Valid Front Matter Format
```
---
title: Document Title
id: document-id
---
```

### Invalid Front Matter Examples
```
---
title: Document Title
id
---
```

```
title: Document Title
id: document-id
```

## Output Format
The tool outputs processing information to stdout in the following format:

### Success Message
```
Processed {count} files. {fixed} files fixed, {valid} already valid, {errors} errors.
```

### Processing Detail
For each file processed:
```
[{status}] {file_path} - {details}
```

Where status is one of: FIXED, VALID, ERROR

## Error Handling
- If a file cannot be read, it's reported as an error and processing continues
- If a file's front matter cannot be parsed, it's reported as an error and processing continues
- If a file cannot be written after fixing, it's reported as an error and processing continues