---

description: "Task list for Branding and UI Enhancement feature implementation"
---

# Tasks: Branding and UI Enhancement

**Input**: Design documents from `/specs/012-branding-ui-enhancement/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: No explicit testing requirements in the feature specification, so we will focus on implementation tasks only.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus project**: `my-book/` at repository root
- Adjusted paths based on plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create project structure per implementation plan in my-book/
- [x] T002 [P] Verify Docusaurus v3.9.2 installation and dependencies in package.json
- [x] T003 [P] Verify Node.js 18+ compatibility and setup

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Prepare custom logo assets with robotics elements in static/img/
- [x] T005 Update docusaurus.config.js with basic site metadata and navigation structure
- [x] T006 Prepare placeholder images for hero section in static/img/
- [x] T007 Set up performance monitoring for image loading times
- [x] T008 Configure accessibility settings for WCAG AA compliance

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - View and Navigate Professional Textbook (Priority: P1) 🎯 MVP

**Goal**: Implement a professional branded interface with a clear book logo featuring robotics elements and consistent design so that users know they're on an official academic resource.

**Independent Test**: Loading the homepage and confirming that the book logo with robotics elements, consistent styling, and proper branding elements appear correctly across different pages without any default Docusaurus template elements.

### Implementation for User Story 1

- [x] T009 [P] [US1] Add custom logo with robotics elements to static/img/robotics-logo.svg
- [x] T010 [US1] Update navbar logo in docusaurus.config.js to use robotics-logo.svg
- [x] T011 [US1] Update site title and tagline in docusaurus.config.js to "Physical AI & Humanoid Robotics"
- [x] T012 [US1] Customize color scheme in src/css/custom.css for academic textbook branding
- [x] T013 [US1] Update favicon to reflect textbook branding in static/img/favicon.ico
- [x] T014 [US1] Replace default Docusaurus header with custom academic-themed header

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Read Content with Properly Rendered Images (Priority: P1)

**Goal**: Ensure images render properly with loading time under 2s instead of showing as alt text, so that students can properly understand visual concepts and diagrams.

**Independent Test**: Visiting pages with images and verifying that all images render correctly with loading time under 2s instead of appearing as alt text or broken image placeholders.

### Implementation for User Story 2

- [x] T015 [P] [US2] Implement image optimization for all textbook images in static/img/
- [x] T016 [US2] Configure image lazy loading in docusaurus.config.js
- [x] T017 [US2] Add placeholder images for broken image fallback in static/img/
- [x] T018 [US2] Update markdown image components to handle loading states
- [x] T019 [US2] Implement image performance monitoring with loading time validation under 2s
- [x] T020 [US2] Add appropriate alt text to all images for accessibility compliance

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Navigate Using Well-Organized Sidebar (Priority: P2)

**Goal**: Ensure the sidebar is properly organized with the Introduction at the top, so that students can easily navigate to the beginning of the textbook.

**Independent Test**: Viewing the sidebar navigation and confirming that the Introduction section appears at the top, followed by subsequent chapters in the expected order.

### Implementation for User Story 3

- [x] T021 [P] [US3] Update sidebars.js to ensure Introduction appears at the top
- [x] T022 [US3] Organize sidebar items in correct order (Introduction, Module 1, Module 2, Module 3, Module 4)
- [x] T023 [US3] Update sidebar styling to match academic textbook theme in src/css/custom.css
- [x] T024 [US3] Ensure sidebar navigation persists across all textbook pages

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: User Story 4 - Experience Professional Footer Information (Priority: P3)

**Goal**: Display properly branded footer information with publisher copyright details for this book, so that users understand the intellectual property rights and publication details.

**Independent Test**: Scrolling to the bottom of any page and verifying that the footer contains publisher copyright information specific to "Physical AI & Humanoid Robotics".

### Implementation for User Story 4

- [x] T025 [P] [US4] Update footer configuration in docusaurus.config.js with publisher copyright
- [x] T026 [US4] Add publisher copyright information to footer: "© [year] Physical AI & Humanoid Robotics. All rights reserved."
- [x] T027 [US4] Style footer to match academic textbook branding in src/css/custom.css

**Checkpoint**: All user stories should now be independently functional

---

## Phase 7: User Story 5 - Avoid Distracting Edit Links (Priority: P3)

**Goal**: Hide "Edit this page" links that distract from the academic experience and suggest the content is still in development, while providing access to the official project repository.

**Independent Test**: Examining pages that previously had edit links and confirming these are no longer visible to readers, and that a GitHub link to the official project repository is available.

### Implementation for User Story 5

- [x] T028 [P] [US5] Configure docusaurus.config.js to hide "Edit this page" links
- [x] T029 [US5] Add GitHub link to the official project repository in navbar or footer
- [x] T030 [US5] Update page metadata to remove edit-related configurations

**Checkpoint**: All user stories should now be independently functional

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T031 [P] Update all page metadata to reflect textbook branding
- [x] T032 [P] Verify mobile responsiveness across all textbook pages
- [x] T033 [P] Finalize color scheme and styling consistency across all pages
- [x] T034 [P] Update hero section with academic-themed content and appropriate images
- [x] T035 [P] Remove all remaining Docusaurus template elements and ensure consistent textbook branding
- [x] T036 [P] Implement fallback strategy for broken images (show placeholder as specified in research)
- [x] T037 Run quickstart.md validation and ensure all requirements meet the specification

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
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 4 (P3)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 5 (P3)**: Can start after Foundational (Phase 2) - No dependencies on other stories

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tasks for User Story 1 together:
Task: "Add custom logo with robotics elements to static/img/robotics-logo.svg"
Task: "Update navbar logo in docusaurus.config.js to use robotics-logo.svg"
Task: "Update site title and tagline in docusaurus.config.js to Physical AI & Humanoid Robotics"
```

---

## Implementation Strategy

### MVP First (User Stories 1 & 2 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. Complete Phase 4: User Story 2
5. **STOP and VALIDATE**: Test User Stories 1 and 2 independently
6. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 (Professional branding) → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 (Image rendering) → Test independently → Deploy/Demo
4. Add User Story 3 (Navigation) → Test independently → Deploy/Demo
5. Add User Story 4 (Footer) → Test independently → Deploy/Demo
6. Add User Story 5 (Edit links) → Test independently → Deploy/Demo
7. Complete Polish phase → Full feature ready

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Branding)
   - Developer B: User Story 2 (Images)
   - Developer C: User Story 3 (Navigation)
   - Developer D: User Story 4 (Footer)
   - Developer E: User Story 5 (Edit links)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [US1], [US2], etc. label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify implementations meet acceptance criteria before moving to next story
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence