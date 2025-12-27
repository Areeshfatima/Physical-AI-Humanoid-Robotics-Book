# Feature Specification: Repair Docusaurus Sidebar and Document IDs

**Feature Branch**: `010-repair-docusaurus-sidebar`
**Created**: Saturday, December 20, 2025
**Status**: Draft
**Input**: User description: "Create a specification to repair Docusaurus sidebar and document IDs. Context: Docusaurus v3.9.2, Docs directory contains invalid, duplicate, or missing IDs; sidebar.ts is out of sync; version loader crashes on intro.md; hidden markdown files may exist (_*.md). Requirements: Scan /docs recursively, Ignore hidden markdown files, Ensure every doc has: unique id, valid title, numeric sidebar_position. Regenerate sidebar.ts automatically from docs tree, Ensure intro.md exists and is first entry, Ensure version loader compatibility, No manual edits by user. Constraints: ASCII only, No special YAML characters, Deterministic output. Deliverables: Spec file under /specs, Sidebar regeneration rules, Validation rules"

## Clarifications

### Session 2025-12-20

- Q: How should conflicting IDs be resolved when a new file has the same ID as an existing file? → A: Maintain existing IDs and add numeric suffixes (e.g., -1, -2) to new conflicting IDs
- Q: What are the performance requirements for processing documentation files? → A: Define specific performance targets for processing time and throughput
- Q: How should deeply nested directory structures be handled in the sidebar? → A: Create nested sidebar categories that mirror the directory structure
- Q: How should the system handle errors during processing? → A: Log errors and continue processing other files
- Q: Should the repair tool support a dry-run mode to preview changes? → A: Yes, provide a dry-run mode to preview changes without applying them

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Docusaurus Documentation Building (Priority: P1)

As a documentation maintainer working with Docusaurus v3.9.2, I need all document IDs and sidebar structure to be properly configured so that the Docusaurus build process completes successfully without errors related to invalid, duplicate, or missing IDs.

**Why this priority**: This is the most critical requirement because invalid IDs and sidebar configuration prevents the documentation from building properly, blocking the entire development and deployment process.

**Independent Test**: Can be fully tested by running `npm run build` in the my-book directory and verifying that the build completes without ID-related or sidebar-related errors.

**Acceptance Scenarios**:

1. **Given** a collection of documentation files with various ID issues (invalid, duplicate, missing), **When** the repair process is executed, **Then** all files have unique, valid IDs and the Docusaurus build completes successfully
2. **Given** a sidebar.ts file that is out of sync with the docs/ directory structure, **When** it's processed by the repair tool, **Then** it's regenerated to match the actual docs structure
3. **Given** hidden markdown files (starting with underscore), **When** the repair process runs, **Then** they are ignored and don't appear in the sidebar

---

### User Story 2 - Docusaurus Server Development (Priority: P2)

As a developer running the Docusaurus development server, I need all document IDs and sidebar positions to be correctly configured so that the server starts and operates without crashes related to the version loader or sidebar issues.

**Why this priority**: Essential for development workflow but secondary to build functionality.

**Independent Test**: Can be tested by starting the Docusaurus development server and verifying it runs without ID-related or version loader crashes.

**Acceptance Scenarios**:

1. **Given** documentation files with invalid or missing IDs, **When** the Docusaurus development server is started, **Then** it operates without version loader crashes after repair
2. **Given** a repaired sidebar.ts file, **When** I access the documentation site, **Then** navigation works correctly according to the docs tree structure
3. **Given** intro.md with ID issues, **When** processed by the repair tool, **Then** the version loader no longer crashes when accessing the file

---

### User Story 3 - Documentation Maintenance (Priority: P3)

As a content author maintaining Docusaurus documentation, I need consistent and valid document IDs and sidebar structure so that I can focus on content creation without encountering ID conflicts or navigation issues.

**Why this priority**: Important for content creation workflow but less critical than basic functionality.

**Independent Test**: Can be tested by verifying that all documentation files in the project have properly formatted IDs and consistent sidebar_position values.

**Acceptance Scenarios**:

1. **Given** documentation files with invalid characters in IDs, **When** processed by the repair tool, **Then** IDs are converted to valid ASCII-only format
2. **Given** documentation files with missing or invalid sidebar_position values, **When** processed by the repair tool, **Then** numeric positions are assigned based on directory structure
3. **Given** new documentation files added to the docs/ directory, **When** the repair process runs, **Then** they are automatically included in the sidebar with proper IDs and positions

---

### Edge Cases

- What happens when a documentation file has an ID that conflicts with an existing one after repair?
- How does the system handle files with non-ASCII characters in titles that need to be converted to IDs?
- What if the intro.md file doesn't exist in the docs directory?
- How does the system handle deeply nested directory structures?
- What if multiple files are missing sidebar_position values?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST scan the my-book/docs/ directory recursively for all .md and .mdx files
- **FR-002**: System MUST ignore hidden markdown files (those starting with underscore, e.g., _*.md)
- **FR-003**: System MUST validate that every documentation file has a unique `id` field in its front matter
- **FR-004**: System MUST validate that every documentation file has a valid `title` field in its front matter
- **FR-005**: System MUST validate that every documentation file has a numeric `sidebar_position` field in its front matter
- **FR-006**: System MUST ensure all IDs are ASCII-only and do not contain special characters
- **FR-007**: System MUST automatically regenerate the sidebar.ts file based on the docs tree structure
- **FR-008**: System MUST ensure intro.md exists and is set as the first entry in the sidebar
- **FR-009**: System MUST handle directory structure by creating appropriate sidebar categories
- **FR-018**: System MUST create nested sidebar categories that mirror the directory structure for deeply nested files
- **FR-010**: System MUST assign numeric sidebar_position values deterministically based on directory structure
- **FR-011**: System MUST validate that all generated IDs are unique across the documentation set
- **FR-012**: System MUST preserve original content in markdown files while only modifying front matter
- **FR-013**: System MUST validate that the version loader compatibility is maintained after repairs
- **FR-014**: System MUST provide a way to preview changes before applying repairs to documentation files
- **FR-020**: System MUST provide a dry-run mode to preview changes without applying them
- **FR-015**: System MUST create a minimal intro.md file if it does not exist
- **FR-016**: System MUST process files in a deterministic order to ensure consistent outputs
- **FR-017**: System MUST resolve ID conflicts by maintaining existing IDs and adding numeric suffixes (e.g., -1, -2) to new conflicting IDs
- **FR-019**: System MUST log errors when encountered and continue processing other files

### Key Entities

- **Documentation File**: A markdown file with a .md or .mdx extension containing YAML front matter with id, title, and sidebar_position
- **Sidebar Configuration**: The sidebar.ts file defining navigation structure for the Docusaurus site
- **Document ID**: A unique identifier for each document that must be ASCII-only and not contain special characters
- **Sidebar Position**: A numeric value determining the order of documents in the sidebar navigation
- **Repair Process**: The automated system that identifies and fixes ID, title, and sidebar issues in documentation files

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of documentation files in my-book/docs/ have unique, valid, ASCII-only IDs after repair process completes
- **SC-002**: 100% of documentation files have valid title and numeric sidebar_position fields after repair process completes
- **SC-003**: The sidebar.ts file is regenerated to match the actual docs tree structure after repair process completes
- **SC-004**: The Docusaurus build process (npm run build) completes successfully without ID-related errors
- **SC-005**: The Docusaurus development server (npm start) runs without version loader crashes
- **SC-006**: intro.md exists and is configured as the first entry in the sidebar
- **SC-007**: All hidden markdown files (starting with underscore) are ignored during processing
- **SC-008**: All documentation content remains intact with no content loss during repairs
- **SC-009**: All generated outputs are deterministic and repeatable
- **SC-010**: Process 1000 documentation files within 30 seconds

### Assumptions

- Docusaurus v3.9.2 is the target framework for the repair process
- The my-book/docs/ directory contains the documentation files to be processed
- The sidebar.ts file needs to be regenerated based on the directory structure
- Node.js and npm are available for running the repair scripts
- The repair process has write access to the relevant files and directories

### Dependencies

- js-yaml: For parsing and generating YAML front matter
- glob: For recursive file scanning
- fs: For file system operations
- Docusaurus v3.9.2: The target framework for documentation