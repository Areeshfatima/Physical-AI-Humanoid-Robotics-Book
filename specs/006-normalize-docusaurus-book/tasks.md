# Tasks: Repository Cleanup & Normalization

**Feature**: `006-normalize-docusaurus-book` - Clean up and normalize repository to a single Docusaurus book with exactly 4 modules and 4 chapters each

## Dependencies

- User Story 2 depends on User Story 1 completion (navigation structure must exist before search can work)
- User Story 3 depends on User Story 1 completion (export functionality requires proper document structure)

## Parallel Execution Examples

- **User Story 1**: Content migration from external docs can be done in parallel across different modules (4 people could migrate one module each)
- **User Story 1**: Chapter cleanup within different modules can be done in parallel
- **User Story 2**: Search optimization across different modules can be done in parallel

## Implementation Strategy

The implementation will follow an MVP approach focusing on User Story 1 first (basic navigation structure) then incrementally adding search and export functionality. The approach will:
1. First set up the correct project structure
2. Migrate essential content to the 4 required modules
3. Clean up navigation and remove stale references
4. Implement advanced features like search and export

---

## Phase 1: Setup

- [ ] T001 Create backup of current documentation structure before modifications
- [ ] T002 Set up development environment with Node.js 18+, npm, and Python 3.8+
- [ ] T003 Install Docusaurus dependencies in /my-book directory
- [ ] T004 Verify current documentation renders correctly with `npm start`

---

## Phase 2: Foundational Tasks

- [X] T005 Identify all valid document IDs in my-book/docs directory
- [ ] T006 Create inventory of external documentation content that should be migrated
- [X] T007 Create inventory of stale content that should be removed (e.g. tutorial-basics, tutorial-extras, module-5)
- [ ] T008 [P] Create content migration scripts to consolidate documentation into correct modules

---

## Phase 3: [US1] Access Organized Educational Content

**User Story Goal**: Create organized Docusaurus textbook with 4 modules and 4 chapters per module

**Independent Test Criteria**: User can browse through the hierarchical structure (modules → chapters → sections) and read content without technical issues

- [X] T009 [P] [US1] Remove stale documentation directories (tutorial-basics, tutorial-extras)
- [X] T010 [P] [US1] Remove module-5 directory which exceeds the 4-module requirement
- [ ] T011 [P] [US1] Migrate valuable content from /docs/vla-system to appropriate chapters in module-4-vla
- [ ] T012 [P] [US1] Migrate valuable content from /gazebo_unity_simulation/documentation to appropriate chapters in module-2-digital-twin
- [ ] T013 [P] [US1] Migrate valuable content from /isaac_sim_vslam_nav/documentation to appropriate chapters in module-3-isaac
- [X] T014 [US1] Update sidebar.js to remove references to stale content (module-5, tutorial directories)
- [X] T015 [US1] Ensure exactly 4 modules exist with proper index files:
  - module-1-ros2 with index.md
  - module-2-digital-twin with index.md
  - module-3-isaac with index.md
  - module-4-vla with index.md
- [X] T016 [P] [US1] Verify each module contains exactly 4 chapter files (chapter-1.md through chapter-4.md)
- [ ] T017 [US1] Standardize frontmatter in all chapter files with required metadata
- [X] T018 [US1] Test navigation between modules and chapters works correctly
- [ ] T019 [US1] Verify all internal links work correctly after cleanup

---

## Phase 4: [US2] Navigate Efficiently Between Related Topics

**User Story Goal**: Implement search functionality to allow users to efficiently locate specific information

**Independent Test Criteria**: Search functionality works across all modules and chapters, returning relevant results for sample queries

- [ ] T020 [US2] Configure Docusaurus search plugin for full-text search across all content
- [ ] T021 [US2] Test search functionality returns relevant results for sample queries like "walking algorithms"
- [ ] T022 [US2] Optimize search index to ensure quick response times (<2 seconds)
- [ ] T023 [US2] Verify search works across all 16 chapters in the 4 modules
- [ ] T024 [US2] Add search result highlighting to improve user experience

---

## Phase 5: [US3] Study Offline or Export Content

**User Story Goal**: Implement export functionality for offline study

**Independent Test Criteria**: Users can generate downloadable versions of individual chapters or selected content

- [ ] T025 [US3] Research and implement PDF export functionality for chapters
- [ ] T026 [US3] Test PDF export for each of the 16 chapters
- [ ] T027 [US3] Verify exported content maintains proper formatting and media
- [ ] T028 [US3] Add export buttons to chapter pages for user access

---

## Phase 6: Polish & Cross-Cutting Concerns

- [X] T029 Update docusaurus.config.js to reflect new document structure
- [ ] T030 Update website title, description, and metadata to reflect Physical AI & Humanoid Robotics content
- [ ] T031 Create comprehensive documentation for the new structure
- [ ] T032 Verify responsive design works on desktop, tablet, and mobile devices
- [ ] T033 Test accessibility features and ensure WCAG 2.1 AA compliance
- [ ] T034 Run performance audit to ensure page load times are under 3 seconds
- [ ] T035 Update README.md to reflect the new organization and navigation
- [ ] T036 Test the complete site build to ensure no broken links or missing content
- [ ] T037 Verify all mathematical formulas, diagrams, and media elements render correctly
- [ ] T038 Create automated tests for continuous integration pipeline
- [ ] T039 Document the final structure for future maintainers