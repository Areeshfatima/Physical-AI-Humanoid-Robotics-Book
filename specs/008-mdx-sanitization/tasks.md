# Tasks: MDX Sanitization

**Feature**: `008-mdx-sanitization` - Scan docs for MDX compilation errors and fix invalid JSX-like syntax

## Dependencies

- Task T003 depends on Task T002 (content scanning must happen before fixing)

## Parallel Execution Examples

- Multiple files can be sanitized in parallel [US1, US2, US3]
- Each user story can be worked on by different team members simultaneously

## Implementation Strategy

The implementation will focus on systematically identifying and fixing MDX compilation errors across all documentation files. The approach will:
1. Scan all documentation files for MDX compilation errors
2. Identify specific instances of invalid JSX-like syntax
3. Convert invalid constructs to pure markdown while preserving meaning
4. Apply minimal edits only to maintain content integrity
5. Verify fixes work correctly in Docusaurus build process

---

## Phase 1: Setup

- [X] T001 Install MDX linting tools for documentation validation
- [X] T002 Set up test environment to validate MDX compilation fixes
- [X] T003 Create backup of documentation files before modifications
- [X] T004 Identify all documentation files in the project

---

## Phase 2: Foundational Tasks

- [ ] T005 Scan all documentation files for MDX compilation errors
- [ ] T006 Catalog all instances of invalid JSX-like syntax in files
- [ ] T007 Create validation script to detect similar patterns in future

---

## Phase 3: [US1] Fix Angle Bracket Issues with Numbers

**User Story Goal**: Correct angle bracket syntax that is misinterpreted as JSX components

**Independent Test Criteria**: Docusaurus build process completes without MDX compilation errors related to angle brackets containing numbers

- [X] T008 [P] [US1] Fix 'ideally <10ms>' in /mnt/e/Hackathon-1/physical-ai-humanoid-robotics/my-book/docs/module-2-digital-twin/chapter-4.md at line 351
- [X] T009 [P] [US1] Fix '- Target: <3 seconds for interactive tasks' in /mnt/e/Hackathon-1/physical-ai-humanoid-robotics/my-book/docs/module-4-vla/chapter-4.md at line 468
- [X] T010 [P] [US1] Fix '- Target: <0.01% (less than 1 in 10,000 hours)' in /mnt/e/Hackathon-1/physical-ai-humanoid-robotics/my-book/docs/module-4-vla/chapter-4.md at line 476
- [X] T011 [P] [US1] Fix '- Speech recognition latency <200ms' in /mnt/e/Hackathon-1/physical-ai-humanoid-robotics/docs/vla-system/index.md at line 45
- [X] T012 [P] [US1] Fix '- LLM processing time <2 seconds' in /mnt/e/Hackathon-1/physical-ai-humanoid-robotics/docs/vla-system/index.md at line 46
- [X] T013 [P] [US1] Fix '- Overall system response <3 seconds' in /mnt/e/Hackathon-1/physical-ai-humanoid-robotics/docs/vla-system/index.md at line 47
- [X] T014 [US1] Test Docusaurus build to confirm angle bracket MDX issues are resolved

---

## Phase 4: [US2] Identify and Fix Other Invalid JSX Constructs

**User Story Goal**: Find and correct any other invalid JSX-like syntax in documentation

**Independent Test Criteria**: All documentation files pass MDX compilation without errors

- [X] T015 [US2] Conduct comprehensive scan for other potential MDX syntax issues
- [X] T016 [P] [US2] Fix any other angle bracket patterns with numbers found in documentation
- [X] T017 [P] [US2] Fix any JSX-like component syntax that's invalid in markdown context
- [X] T018 [US2] Verify all code blocks are properly escaped and not misinterpreted as JSX
- [X] T019 [US2] Test full Docusaurus build to confirm all MDX issues are resolved

---

## Phase 5: [US3] Validate Content Preservation

**User Story Goal**: Ensure all fixes preserve the original meaning and structure of content

**Independent Test Criteria**: Documentation content remains accurate and readable after fixes

- [X] T020 [US3] Review all changes to ensure technical accuracy is maintained
- [X] T021 [US3] Verify that converted content maintains original meaning
- [X] T022 [US3] Confirm formatting and structure of documents remain appropriate
- [X] T023 [US3] Validate that performance metrics and technical specifications are preserved
- [X] T024 [US3] Ensure all code examples and technical descriptions remain correct

---

## Phase 6: Polish & Cross-Cutting Concerns

- [X] T025 Update documentation guidelines to prevent MDX issues in future contributions
- [X] T026 Add MDX validation to the CI/CD pipeline
- [X] T027 Create documentation for handling angle brackets and special characters
- [X] T028 Perform final validation with full Docusaurus build
- [X] T029 Document the sanitization process for future maintainers
- [X] T030 Archive backup files after successful validation