---

description: "Task list for Markdown Front Matter Fix feature implementation - focused on scanning, identifying, and fixing front matter issues"
---

# Tasks: Markdown Front Matter Fix - Front Matter Scanner

**Input**: Design documents from `/specs/001-markdown-front-matter-fix/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, quickstart.md

**Tests**: Testing strategy included as per quickstart.md and feature specification.
**Focus**: Scan my-book/docs/**/*.md and .mdx - Identify files with missing or invalid YAML front matter - Ensure front matter starts at line 1 with --- - Quote all string values - Remove unsupported characters from unquoted values - Ensure each doc has id and title

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Markdown processing**: `my-book/docs/` for documentation files, `scripts/` for processing utilities

# Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and environment setup

- [ ] T001 Install required dependencies (js-yaml for YAML processing)
- [ ] T002 [P] Set up glob pattern matching to scan my-book/docs/**/*.md and my-book/docs/**/*.mdx
- [ ] T003 Create backup directory for original files preservation

---

# Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T004 [P] Implement file scanner to identify all .md and .mdx files in my-book/docs/
- [ ] T005 [P] Create function to detect if file starts with YAML front matter (--- on line 1)
- [ ] T006 [P] Create function to detect missing YAML front matter in files
- [ ] T007 [P] Create function to detect invalid YAML syntax in existing front matter
- [ ] T008 Create function to parse YAML content from files that have valid front matter

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

# Phase 3: User Story 1 - Docusaurus Documentation Building (Priority: P1) 🎯 MVP

**Goal**: Ensure all documentation files have valid YAML front matter so that the Docusaurus build process completes successfully without errors

**Independent Test**: Can be fully tested by running `npm run build` in the my-book directory and verifying that the build completes without front matter parsing errors.

### Implementation for User Story 1

- [ ] T009 [P] [US1] Implement function to ensure front matter starts at line 1 with ---
- [ ] T010 [P] [US1] Create function to validate required fields (id and title exist) in front matter
- [ ] T011 [P] [US1] Create function to add missing required fields (id and title) to front matter
- [ ] T012 [US1] Implement logic to ensure front matter starts at line 1 of file
- [ ] T013 [US1] Create function to generate appropriate title based on filename if missing
- [ ] T014 [US1] Create function to generate appropriate id based on filename if missing
- [ ] T015 [US1] Test repair process with sample file from my-book/docs/image-handling.md
- [ ] T016 [US1] Validate YAML parsing: Ensure all processed files have syntactically correct YAML front matter
- [ ] T017 [US1] Run npm run build: Verify the Docusaurus build process completes successfully
- [ ] T018 [US1] Confirm repair tool processes only .md and .mdx files in my-book/docs/

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

# Phase 4: User Story 2 - Documentation Authoring Experience (Priority: P2)

**Goal**: Ensure consistent and valid front matter across all documentation files so that authors can focus on writing content without encountering build errors

**Independent Test**: Can be tested by verifying that all documentation files in the project have properly formatted front matter that meets Docusaurus requirements.

### Implementation for User Story 2

- [ ] T019 [P] [US2] Implement function to quote all string values in front matter
- [ ] T020 [P] [US2] Create function to remove unsupported characters from unquoted values
- [ ] T021 [US2] Process all files in my-book/docs/ to ensure consistent field formatting
- [ ] T022 [US2] Validate that all string values are properly quoted in front matter
- [ ] T023 [US2] Verify that special characters are handled and escaped properly
- [ ] T024 [US2] Content preservation check: Ensure no body content is modified during repairs
- [ ] T025 [US2] Check that document structure and IDs remain unchanged during repairs
- [ ] T026 [US2] Verify all docs have valid YAML front matter after processing
- [ ] T027 [US2] Test repair functionality on files with special characters in values
- [ ] T028 [US2] Document the consistent front matter format for future authors

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

# Phase 5: User Story 3 - Docusaurus Server Development (Priority: P3)

**Goal**: Ensure all documentation files have valid front matter so that the Docusaurus development server starts and operates without front matter-related errors

**Independent Test**: Can be tested by starting the Docusaurus development server and verifying it runs without front matter parsing errors.

### Implementation for User Story 3

- [ ] T029 [US3] Run npm start after repair: Verify the Docusaurus development server operates without front-matter parsing errors
- [ ] T030 [US3] Test development server with all repaired documentation files
- [ ] T031 [US3] Verify that files with previously invalid front matter now work in development mode
- [ ] T032 [US3] Validate that all documentation files are properly indexed by Docusaurus
- [ ] T033 [US3] Perform final validation of 100% of documentation files in my-book/docs/
- [ ] T034 [US3] Test edge cases: files with malformed YAML that cannot be automatically corrected
- [ ] T035 [US3] Test edge cases: files with special characters that need to be escaped
- [ ] T036 [US3] Test edge cases: files with multiple YAML blocks or conflicting front matter sections
- [ ] T037 [US3] Confirm all success criteria from spec are met (no content loss, structure preservation, etc.)
- [ ] T038 [US3] Execute comprehensive validation scan of all processed files

**Checkpoint**: All user stories should now be independently functional

---

# Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T039 [P] Add error handling for files that cannot be processed
- [ ] T040 [P] Add progress reporting during file scanning and processing
- [ ] T041 [P] Create summary report of files processed, fixed, and unchanged
- [ ] T042 Add performance optimization for processing large numbers of files
- [ ] T043 Final comprehensive test with npm start and npm run build
- [ ] T044 Create usage documentation for the repair tool

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
Task: "Implement function to ensure front matter starts at line 1 with ---"
Task: "Create function to validate required fields (id and title exist) in front matter"
Task: "Create function to add missing required fields (id and title) to front matter"
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
- Focus: Scan my-book/docs/**/*.md and .mdx, identify files with missing/invalid YAML front matter, ensure proper formatting, quote all string values, remove unsupported characters, ensure id and title exist