---

description: "Task list for Docusaurus CLI setup feature implementation - focused on fixing npm start command and ensuring local CLI installation"
---

# Tasks: Docusaurus CLI Setup - Tooling Fix

**Input**: Design documents from `/specs/001-docusaurus-cli-setup/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, quickstart.md

**Tests**: Testing strategy included as per quickstart.md and feature specification.
**Focus**: Inspect my-book/package.json - Verify presence of @docusaurus/core and preset-classic - Install missing Docusaurus dependencies locally - Ensure npm scripts reference local CLI - Do not modify docs or content.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `my-book/` for the Docusaurus documentation site
- Paths shown below based on plan.md structure for Docusaurus project

# Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and environment audit

- [ ] T001 Audit current my-book package.json for Docusaurus dependencies
- [ ] T002 [P] Verify Node.js 18+ installation on development machine
- [ ] T003 [P] Check current npm and npx availability
- [ ] T004 [P] Examine current state of node_modules directory in my-book

---

# Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T005 [P] Test current npm start command in my-book directory (should fail with "docusaurus: not found")
- [ ] T006 [P] Check if docusaurus CLI is properly available via npx docusaurus --version
- [ ] T007 [P] Verify docusaurus.config.js is properly configured
- [ ] T008 [P] Validate that @docusaurus/core and @docusaurus/preset-classic are properly installed
- [ ] T009 [P] Check for any corrupted packages in node_modules

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

# Phase 3: User Story 1 - Docusaurus Development Environment (Priority: P1) 🎯 MVP

**Goal**: Enable developers to run the Docusaurus application with a simple command to view and test documentation locally

**Independent Test**: Can be fully tested by running `npm start` and accessing the locally hosted textbook website to verify that it loads without errors and all pages are accessible.

### Implementation for User Story 1

- [ ] T010 [P] [US1] Verify @docusaurus/core is installed as dependency in my-book/package.json
- [ ] T011 [P] [US1] Verify @docusaurus/preset-classic is installed as dependency in my-book/package.json
- [ ] T012 [P] [US1] Check that docusaurus CLI is available in devDependencies in my-book/package.json
- [ ] T013 [US1] Install missing Docusaurus CLI if not present: npm install --save-dev @docusaurus/cli
- [ ] T014 [US1] Confirm npm start script properly references local docusaurus CLI in my-book/package.json
- [ ] T015 [US1] Test npm install command works without errors in my-book directory
- [ ] T016 [US1] Test npm start command runs development server without "docusaurus: not found" error
- [ ] T017 [US1] Verify development server runs on default URL (localhost:3000)
- [ ] T018 [US1] Verify documentation pages load without errors
- [ ] T019 [US1] Confirm startup time is under 30 seconds as specified

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

# Phase 4: User Story 2 - Docusaurus CLI and Dependencies Installation (Priority: P2)

**Goal**: Enable new team members to install Docusaurus and its required dependencies to contribute to the textbook project

**Independent Test**: Can be tested by installing all required packages and verifying they're properly configured in the project.

### Implementation for User Story 2

- [ ] T020 [P] [US2] Verify Docusaurus CLI is installed locally as dev dependency in my-book/package.json
- [ ] T021 [P] [US2] Test npx docusaurus --version command works properly
- [ ] T022 [US2] Test npx docusaurus --help command works properly
- [ ] T023 [US2] Validate all core Docusaurus packages are properly installed and compatible
- [ ] T024 [US2] Test docusaurus build command works properly
- [ ] T025 [US2] Verify installation works consistently across different operating systems (Windows, macOS, Linux)
- [ ] T026 [US2] Document the proper installation process for new team members

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

# Phase 5: User Story 3 - Tooling-Only Fix (Priority: P3)

**Goal**: Ensure Docusaurus setup only affects tooling aspects without changing existing content or structure of the textbook

**Independent Test**: Can be verified by confirming that no content or documentation structure is changed as part of the tooling fix.

### Implementation for User Story 3

- [ ] T027 [US3] Verify no changes were made to existing documentation files in my-book/docs/
- [ ] T028 [US3] Verify sidebar configuration remains unchanged in my-book/sidebars.js
- [ ] T029 [US3] Verify no changes to blog content in my-book/blog/
- [ ] T030 [US3] Verify no changes to static assets in my-book/static/
- [ ] T031 [US3] Verify no changes to custom React components in my-book/src/
- [ ] T032 [US3] Confirm only package.json and related config files were potentially modified
- [ ] T033 [US3] Verify existing documentation remains unchanged when running npm start

**Checkpoint**: All user stories should now be independently functional

---

# Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T034 [P] Update README with Docusaurus setup instructions
- [ ] T035 [P] Verify hot reload functionality works properly
- [ ] T036 [P] Run quickstart validation tests (npm install → npm start → localhost verification)
- [ ] T037 Cross-platform testing on different operating systems
- [ ] T038 Verify no content or structure was changed during tooling fix
- [ ] T039 Final verification of all success criteria from spec

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
- Multiple dependency checks within a story marked [P] can run in parallel

---

## Parallel Example: User Story 1

```bash
# Launch all dependency checks for User Story 1 together:
Task: "Verify @docusaurus/core is installed as dependency in my-book/package.json"
Task: "Verify @docusaurus/preset-classic is installed as dependency in my-book/package.json"
Task: "Check that docusaurus CLI is available in devDependencies in my-book/package.json"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently with npm start
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
- Focus: Tooling fix only, no content changes