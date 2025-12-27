# Data Model: Repair Docusaurus Sidebar and Document IDs

## Core Entities

### DocumentationFile
- **id**: string - Unique identifier for the document (must be ASCII-only)
- **title**: string - Display title for the document
- **sidebar_position**: number - Numeric position in the sidebar
- **filepath**: string - Full path to the markdown file
- **relative_path**: string - Path relative to docs directory
- **directory_structure**: string - Path segments for creating sidebar categories
- **has_front_matter**: boolean - Whether the file has a front matter block
- **required_fields**: array - List of required fields (id, title, sidebar_position)

### SidebarEntry
- **type**: string - Entry type (e.g., "category", "doc")
- **label**: string - Display name for the sidebar entry
- **items**: array - Child items in the category (for nested structures)
- **link**: object - Link configuration containing docId
- **collapsed**: boolean - Whether the category is collapsed by default
- **collapsible**: boolean - Whether the category can be collapsed

### ProcessingResult
- **filepath**: string - Path of the processed file
- **status**: string - Status of the processing ('repaired', 'already_valid', 'error', 'ignored')
- **changes**: array - List of changes made to the file
- **errors**: array - List of errors encountered during processing
- **preview_only**: boolean - Whether this was a dry-run preview

## Relationships
- A DocumentationFile contains zero or one YAML front matter block
- A SidebarEntry may contain multiple DocumentationFile objects as items
- A SidebarEntry may contain multiple other SidebarEntry objects for nested structures
- Multiple DocumentationFile entities form the hierarchical structure of sidebar entries
- ProcessingResult represents the outcome of processing a DocumentationFile

## State Transitions
- DocumentationFile state: {no_front_matter, invalid_front_matter, valid_front_matter}
- ProcessingResult state: {pending, processing, completed, error}
- SidebarEntry state: {unassigned, assigned, validated}

## Validation Rules
- DocumentationFile.id must be unique across the entire documentation set
- DocumentationFile.id must contain only ASCII characters with no special YAML characters
- DocumentationFile.sidebar_position must be numeric
- DocumentationFile.title must be present and non-empty
- SidebarEntry.label must match the corresponding DocumentationFile.title
- DocumentationFile.content must remain unchanged during processing (only front matter modified)
- Sidebar structure must mirror the directory structure of documentation files

## Conflict Resolution
- When duplicate IDs are detected, preserve existing ID and add numeric suffix to new conflicting ID
- When directory structure is deeply nested, create corresponding nested categories in sidebar
- When intro.md doesn't exist, create with minimal required fields