# Data Model: Markdown Front Matter Fix

## Core Entities

### MarkdownDocument
- **id**: string - File path relative to docs directory
- **content**: string - Full content of the markdown file
- **frontMatter**: object - Parsed YAML front matter object
- **hasFrontMatter**: boolean - Whether the file has a front matter block
- **isValidYaml**: boolean - Whether the front matter is valid YAML
- **requiredFields**: array - List of required Docusaurus fields (title, id)

### FrontMatter
- **title**: string - Title of the document (required)
- **id**: string - Unique identifier for the document (required)
- **description**: string - Optional description of the document
- **otherFields**: object - Any other custom fields in the front matter

### ProcessingResult
- **filePath**: string - Path of the processed file
- **status**: string - Status of the processing ('fixed', 'already_valid', 'error')
- **changes**: array - List of changes made to the file
- **errors**: array - List of errors encountered during processing

## Relationships
- A MarkdownDocument contains zero or one FrontMatter block
- ProcessingResult represents the outcome of processing a MarkdownDocument
- Multiple MarkdownDocument entities may be processed in a single run

## State Transitions
- MarkdownDocument state: {no_front_matter, invalid_front_matter, valid_front_matter}
- Processing moves documents from {no_front_matter, invalid_front_matter} → valid_front_matter

## Validation Rules
- FrontMatter must contain 'title' and 'id' fields
- FrontMatter must be valid YAML syntax
- FrontMatter must be properly delimited with '---' at start and end
- Document body content must remain unchanged during processing