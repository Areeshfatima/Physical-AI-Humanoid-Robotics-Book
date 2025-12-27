---

description: "Task list for Markdown Front Matter Fix feature implementation"
---

# Tasks: Markdown Front Matter Fix

**Input**: Design documents from `/specs/001-markdown-front-matter-fix/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, quickstart.md

**Tests**: Testing strategy included as per quickstart.md and feature specification.
**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Markdown processing**: `my-book/docs/` for documentation files, `scripts/` for processing utilities

# Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and environment setup

- [x] T001 Set up project directory structure for front matter repair tool
- [x] T002 [P] Install required dependencies (fs, yaml, glob for Node.js processing)
- [x] T003 [P] Verify Node.js environment is available for processing
- [x] T004 Create scripts directory for the repair tool

---

# Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T005 [P] Implement YAML parsing utilities in scripts/utils/yaml-parser.js
- [x] T006 [P] Create file scanning function to find all .md and .mdx files in my-book/docs/
- [x] T007 [P] Implement front matter detection logic in scripts/utils/frontmatter.js
- [x] T008 [P] Create backup utility to preserve original files before modification
- [x] T009 Create initial front matter repair function skeleton

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

# Phase 3: User Story 1 - Docusaurus Documentation Building (Priority: P1) 🎯 MVP

**Goal**: Ensure all documentation files have valid YAML front matter so that the Docusaurus build process completes successfully without errors

**Independent Test**: Can be fully tested by running `npm run build` in the my-book directory and verifying that the build completes without front matter parsing errors.

### Implementation for User Story 1

- [x] T010 [P] [US1] Implement missing front matter detection in scripts/utils/validator.js
- [x] T011 [P] [US1] Create function to extract content from markdown files in scripts/utils/content-extractor.js
- [x] T012 [US1] Implement YAML validation function to check syntax in scripts/utils/validator.js
- [x] T013 [US1] Create function to add minimal required front matter (id, title) to files missing front matter
- [x] T014 [US1] Create function to repair invalid YAML syntax in existing front matter blocks
- [x] T015 [US1] Generate appropriate title and id based on filename for files missing front matter
- [x] T016 [US1] Implement the core repair process in scripts/repair-front-matter.js
- [x] T017 [US1] Validate YAML parsing: Ensure all processed files have syntactically correct YAML front matter
- [x] T018 [US1] Run npm run build: Verify the Docusaurus build process completes successfully
- [x] T019 [US1] Test repair process with files from my-book/docs/image-handling.md

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

# Phase 4: User Story 2 - Documentation Authoring Experience (Priority: P2)

**Goal**: Ensure consistent and valid front matter across all documentation files so that authors can focus on writing content without encountering build errors

**Independent Test**: Can be tested by verifying that all documentation files in the project have properly formatted front matter that meets Docusaurus requirements.

### Implementation for User Story 2

- [x] T020 [US2] Implement consistent front matter formatting in scripts/repair-front-matter.js
- [x] T021 [US2] Add sidebar_position field to front matter when missing
- [x] T022 [P] [US2] Apply defensive quoting strategy to all string values in front matter
- [x] T023 [US2] Process all files in my-book/docs/ to ensure consistency
- [x] T024 [US2] Validate field requirements (id, title, sidebar_position) across all files
- [x] T025 [US2] Check that document structure and IDs remain unchanged during repairs
- [x] T026 [US2] Content preservation check: Ensure no body content is modified during repairs
- [x] T027 [US2] Test repair functionality on a subset of files to verify consistency
- [x] T028 [US2] Verify all docs have valid YAML front matter after processing
- [x] T029 [US2] Document the consistent front matter format for future authors

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

# Phase 5: User Story 3 - Docusaurus Server Development (Priority: P3)

**Goal**: Ensure all documentation files have valid front matter so that the Docusaurus development server starts and operates without front matter-related errors

**Independent Test**: Can be tested by starting the Docusaurus development server and verifying it runs without front matter parsing errors.

### Implementation for User Story 3

- [x] T030 [US3] Run npm start after repair: Verify the Docusaurus development server operates without front-matter parsing errors
- [x] T031 [US3] Test development server with repaired documentation files
- [x] T032 [US3] Verify that files with previously invalid front matter now work in development mode
- [x] T033 [US3] Test hot reload functionality with repaired front matter
- [x] T034 [US3] Validate that all documentation files are properly indexed by Docusaurus
- [x] T035 [US3] Perform final validation of 100% of documentation files in my-book/docs/
- [x] T036 [US3] Test edge cases: files with malformed YAML that cannot be automatically corrected
- [x] T037 [US3] Test edge cases: files with special characters that need to be escaped
- [x] T038 [US3] Test edge cases: files with multiple YAML blocks or conflicting front matter sections
- [x] T039 [US3] Confirm all success criteria from spec are met (no content loss, structure preservation, etc.)

**Checkpoint**: All user stories should now be independently functional

---

# Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T040 [P] Update README with front matter repair tool usage instructions
- [x] T041 [P] Add error handling and logging to the repair script
- [x] T042 [P] Create a preview function to show changes before applying repairs
- [x] T043 [P] Add performance optimization to process files efficiently
- [x] T044 Add progress reporting during file processing
- [x] T045 Final comprehensive test with npm start and npm run build
- [x] T046 Verify all 001-markdown-front-matter-fix artifacts are properly documented

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
# Launch all utility implementations for User Story 1 together:
Task: "Implement missing front matter detection in scripts/utils/validator.js"
Task: "Create function to extract content from markdown files in scripts/utils/content-extractor.js"
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
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
- Focus: Front matter fixes only, no content changes