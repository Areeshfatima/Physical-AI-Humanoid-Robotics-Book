---

description: "Tasks for implementing Docusaurus sidebar and document ID repair tool"
---

# Tasks: Repair Docusaurus Sidebar and Document IDs

**Input**: Design documents from `/specs/010-repair-docusaurus-sidebar/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, quickstart.md

**Tests**: Testing strategy included as per quickstart.md and feature specification.
**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Markdown processing**: `my-book/docs/` for documentation files, `scripts/` for processing utilities
- **Sidebar generation**: `src/utils/` for processing utilities, `my-book/sidebars.js` for output

# Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and environment setup

- [X] T001 [P] Set up project directory structure for front matter and sidebar repair tool
- [X] T002 [P] Install required dependencies (fs, yaml, glob, js-yaml for Node.js processing)
- [X] T003 [P] Verify Node.js environment is available for processing
- [X] T004 Create scripts directory for the repair tool

---

# Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T005 [P] Implement file scanner to recursively find all .md and .mdx files in my-book/docs/
- [X] T006 [P] Create front matter detection utility in src/utils/frontmatter-processor.js
- [X] T007 [P] Create ID generator utility in src/utils/id-generator.js
- [X] T008 [P] Create sidebar generator utility in src/utils/sidebar-generator.js
- [X] T009 [P] Create conflict resolver utility in src/utils/conflict-resolver.js
- [X] T010 [P] Create validator utility in src/utils/validator.js
- [X] T011 Create initial repair function skeleton

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

# Phase 3: User Story 1 - Docusaurus Documentation Building (Priority: P1) 🎯 MVP

**Goal**: Ensure all documentation files have valid document IDs and sidebar structure so that the Docusaurus build process completes successfully without errors related to invalid, duplicate, or missing IDs

**Independent Test**: Can be fully tested by running `npm run build` in the my-book directory and verifying that the build completes without ID-related or sidebar-related errors.

### Implementation for User Story 1

- [X] T012 [P] [US1] Implement duplicate and missing ID detection in src/utils/validator.js
- [X] T013 [P] [US1] Create function to extract content from markdown files in src/utils/frontmatter-processor.js
- [X] T014 [US1] Implement validation function to check for unique, valid, ASCII-only IDs in src/utils/validator.js
- [X] T015 [US1] Create function to add missing required fields (id, title, sidebar_position) to files missing front matter
- [X] T016 [US1] Create function to repair invalid YAML syntax in existing front matter blocks
- [X] T017 [P] [US1] Generate appropriate id, title and sidebar_position based on filename for files missing front matter
- [X] T018 [P] [US1] Implement the core repair process in scripts/repair-sidebar-ids.js
- [X] T019 [US1] Validate YAML parsing: Ensure all processed files have syntactically correct YAML front matter
- [X] T020 [US1] Validate ID uniqueness: Ensure all processed files have unique IDs across documentation set
- [X] T021 [US1] Run npm run build: Verify the Docusaurus build process completes successfully
- [X] T022 [US1] Validate that hidden markdown files (starting with underscore) are ignored during processing

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

# Phase 4: User Story 2 - Docusaurus Server Development (Priority: P2)

**Goal**: Ensure all document IDs and sidebar positions are correctly configured so that the server starts and operates without crashes related to the version loader or sidebar issues

**Independent Test**: Can be tested by starting the Docusaurus development server and verifying it runs without ID-related or version loader crashes.

### Implementation for User Story 2

- [X] T023 [P] [US2] Implement consistent ID and title generation in src/utils/id-generator.js
- [X] T024 [US2] Ensure all sidebar_position values are numeric and properly assigned in src/utils/validator.js
- [X] T025 [P] [US2] Process all files in my-book/docs/ to ensure all have proper positions
- [X] T026 [US2] Validate field requirements (id, title, sidebar_position) across all files
- [X] T027 [US2] Check that document structure and IDs remain unchanged during repairs
- [X] T028 [US2] Content preservation check: Ensure no body content is modified during repairs
- [X] T029 [US2] Run development server with repaired documentation: Verify no version loader crashes
- [X] T030 [US2] Test intro.md file specifically: Ensure it exists and has proper configuration
- [X] T031 [US2] Validate version loader compatibility after repairs

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

# Phase 5: User Story 3 - Documentation Maintenance (Priority: P3)

**Goal**: Ensure consistent and valid document IDs and sidebar structure so that content authors can focus on content creation without encountering ID conflicts or navigation issues

**Independent Test**: Can be tested by verifying that all documentation files in the project have properly formatted IDs and consistent sidebar_position values.

### Implementation for User Story 3

- [X] T032 [US3] Process new documentation files: Include any newly added files in repairs
- [X] T033 [US3] Implement ASCII-only ID enforcement in src/utils/validator.js
- [X] T034 [P] [US3] Apply defensive quoting strategy to all string values in front matter
- [X] T035 [P] [US3] Create nested sidebar categories that mirror directory structure
- [X] T036 [US3] Handle deeply nested directory structures with appropriate sidebar categories
- [X] T037 [US3] Process files with invalid characters in IDs to convert to valid ASCII-only format
- [X] T038 [US3] Assign numeric positions based on directory structure
- [X] T039 [US3] Verify all docs have valid YAML front matter after processing
- [X] T040 [US3] Validate that new documentation files are automatically included in the sidebar
- [X] T041 [US3] Confirm all success criteria from spec are met (no content loss, structure preservation, etc.)

**Checkpoint**: All user stories should now be independently functional

---

# Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T042 [P] Update README with repair tool usage instructions
- [X] T043 [P] Add error handling and logging to the repair script (log errors and continue processing other files)
- [X] T044 [P] Create dry-run functionality to preview changes without applying them in scripts/dry-run-mode.js
- [X] T045 [P] Add progress reporting during file processing and performance tracking
- [X] T046 Add backup functionality to preserve original files before modification
- [X] T047 Final comprehensive test with npm start and npm run build
- [X] T048 Verify all 010-repair-docusaurus-sidebar artifacts are properly documented

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Multiple utility functions within a story marked [P] can run in parallel

---

## Parallel Example: User Story 1

```bash
# Launch all validation functions for User Story 1 together:
Task: "Implement duplicate and missing ID detection in src/utils/validator.js"
Task: "Create function to extract content from markdown files in src/utils/frontmatter-processor.js"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently with npm run build
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Focus: Scan my-book/docs/**/*.md and .mdx - Identify files with invalid/duplicate/missing IDs and sidebar positions - Ensure all docs have unique, valid, ASCII-only IDs - Regenerate sidebar.ts from directory structure - Handle deeply nested directories properly