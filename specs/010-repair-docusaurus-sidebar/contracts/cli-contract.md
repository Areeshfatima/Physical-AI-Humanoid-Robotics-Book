# Command Line Interface Contract: Sidebar and Document ID Repair Tool

## Overview
This document specifies the command-line interface for the Docusaurus sidebar and document ID repair tool.

## Command Structure
```
node scripts/repair-sidebar-ids.js [options]
```

## Options

### --help
Displays help information about the tool and its options.

**Type**: Flag
**Required**: No
**Default**: false

### --dry-run
Performs a dry run without modifying any files.

**Type**: Flag
**Required**: No
**Default**: false
**Behavior**: Reports what changes would be made without actually modifying files.

### --docs-dir [path]
Specifies the documentation directory to process (default is my-book/docs/).

**Type**: String with argument
**Required**: No
**Default**: "my-book/docs/"
**Validation**: Must be a valid directory path containing markdown files.

### --backup
Creates backup copies of files before modifying them.

**Type**: Flag
**Required**: No
**Default**: false
**Behavior**: When enabled, the tool creates .bak files before modifying any markdown files.

### --performance-test
Runs a performance test to measure processing speed.

**Type**: Flag
**Required**: No
**Default**: false
**Behavior**: Measures time taken to process all files and reports performance metrics.

### --verbose
Provides verbose output with detailed logging.

**Type**: Flag
**Required**: No
**Default**: false
**Behavior**: Outputs detailed information about each processing step.

### --log-errors
Logs errors to a file instead of stopping execution.

**Type**: Flag
**Required**: No
**Default**: false
**Behavior**: Continues processing other files when errors occur in specific files.

## Exit Codes
- 0: Success, all operations completed without errors
- 1: General error occurred during processing
- 2: Invalid arguments provided
- 3: Performance requirements not met (too slow)
- 4: ID validation failed (non-unique IDs found)

## Input Format
The tool processes markdown files (.md and .mdx extensions) that may contain YAML front matter.

### Valid Front Matter Format
```
---
id: unique-document-id
title: Document Title
sidebar_position: 2
---
```

### Invalid Front Matter Examples
```
---
title: Document Title
sidebar_position: 2
---
```

```
id: duplicate-id
title: Document Title
sidebar_position: abc
```

## Output Format
The tool outputs processing information to stdout in the following format:

### Success Message
```
Processed {count} files. {repaired} files repaired, {valid} already valid, {errors} errors.
Performance: Processed {count} files in {seconds}s ({rate} files/s).
```

### Processing Detail
For each file processed:
```
[{status}] {file_path} - {details}
```

Where status is one of: REPAIRED, VALID, ERROR, IGNORED

## Error Handling
- If a file cannot be read, it's reported as an error and processing continues
- If a file's front matter cannot be parsed, it's reported as an error and processing continues
- If a file cannot be written after fixing, it's reported as an error and processing continues
- If duplicate IDs are found after processing, the tool attempts to resolve them by adding numeric suffixes
- If the sidebar.ts file cannot be regenerated, the error is logged and the process continues

## Performance Requirements
- Tool must process 1000 documentation files within 30 seconds
- Memory usage should remain reasonable during large batch processing
- The tool should gracefully handle deeply nested directory structures
- Processing should maintain determinism - the same input should produce the same output