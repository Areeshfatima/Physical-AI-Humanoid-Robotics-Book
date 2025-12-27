# Implementation Plan: [FEATURE]

**Branch**: `[###-feature-name]` | **Date**: [DATE] | **Spec**: [link]
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a cleanup and normalization of the repository structure to consolidate all educational content under a single Docusaurus textbook at `/my-book/docs` with exactly 4 modules and 4 chapters per module. This involves removing non-book documentation, consolidating content from external documentation directories, and ensuring the main textbook follows proper academic standards with 4 modules covering: ROS2 Fundamentals, Digital Twins (Gazebo & Unity), AI-Robot Brain (NVIDIA Isaac), and Vision-Language-Action (VLA) Models. The implementation will ensure proper navigation, content quality, and alignment with the Physical AI & Humanoid Robotics curriculum.

## Technical Context

<!--
  ACTION REQUIRED: Replace the content in this section with the technical details
  for the project. The structure here is presented in advisory capacity to guide
  the iteration process.
-->

**Language/Version**: JavaScript/TypeScript, Node.js 18+ (for Docusaurus), Python 3.10 (for content processing scripts)
**Primary Dependencies**: Docusaurus v3.9.2, React, Node.js, OpenAI SDK (for RAG functionality), FS-extra (for file operations)
**Storage**: Git-based Markdown files for content in `/my-book/docs/`, with additional content potentially migrated from external documentation directories
**Testing**: Jest for frontend, manual testing for content accuracy, accessibility testing using axe-core
**Target Platform**: Web application (deployable to GitHub Pages and Vercel)
**Project Type**: Web application to support the textbook interface and RAG chatbot functionality
**Performance Goals**: Support 1000+ concurrent users, search response time <2s (95% of queries), page load time <3s (90% of views)
**Constraints**: Must meet WCAG 2.1 AA compliance standards, support rich media content (images, diagrams, videos, equations), offline export capability (PDF)
**Scale/Scope**: 4 modules with 4 chapters each (16 total chapters), support for RAG chatbot queries against textbook content

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Compliance Status
- ✅ AI-Native Development: Docusaurus textbook will be AI-assisted and leverage Claude for content creation and restructuring
- ✅ RAG Integration: Backend with FastAPI, Qdrant, and OpenAI SDK will support RAG chatbot functionality that works with the normalized content structure
- ✅ Personalization Features: Optional as per constitution
- ✅ Quality Standards: Implementation will follow best practices with clear documentation and academic rigor
- ✅ Reproducible Code: Automated deployment to GitHub Pages and Vercel will ensure reproducibility with normalized content structure
- ✅ Content Constraints: Will adhere to 3000-5000 words requirement with academic rigor in the cleaned up textbook
- ✅ Technology Stack: Uses required technologies (Docusaurus, OpenAI SDKs, FastAPI, Neon Postgres, Qdrant) for the RAG functionality on the normalized content

### Gates Passed
All constitutional requirements are satisfied by the planned implementation.

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
my-book/
├── docs/
│   ├── module-1-ros2/
│   │   ├── index.md
│   │   ├── chapter-1.md
│   │   ├── chapter-2.md
│   │   ├── chapter-3.md
│   │   └── chapter-4.md
│   ├── module-2-digital-twin/
│   │   ├── index.md
│   │   ├── chapter-1.md
│   │   ├── chapter-2.md
│   │   ├── chapter-3.md
│   │   └── chapter-4.md
│   ├── module-3-isaac/
│   │   ├── index.md
│   │   ├── chapter-1.md
│   │   ├── chapter-2.md
│   │   ├── chapter-3.md
│   │   └── chapter-4.md
│   └── module-4-vla/
│       ├── index.md
│       ├── chapter-1.md
│       ├── chapter-2.md
│       ├── chapter-3.md
│       └── chapter-4.md
├── src/
│   ├── components/
│   ├── pages/
│   └── theme/
├── static/
├── docusaurus.config.js
├── sidebars.js
├── package.json
└── README.md

backend/
├── src/
│   ├── models/
│   │   ├── module.py
│   │   ├── chapter.py
│   │   ├── user.py
│   │   └── rag.py
│   ├── services/
│   │   ├── content_service.py
│   │   ├── rag_service.py
│   │   └── user_service.py
│   ├── api/
│   │   ├── v1/
│   │   │   ├── content.py
│   │   │   ├── rag.py
│   │   │   └── auth.py
│   │   └── main.py
│   └── utils/
│       ├── auth.py
│       └── validators.py
├── tests/
│   ├── unit/
│   ├── integration/
│   └── contract/
├── requirements.txt
├── Dockerfile
└── docker-compose.yml
```

**Structure Decision**: Web application with separate frontend (Docusaurus) and backend (FastAPI) components to support the textbook interface and RAG chatbot functionality. The frontend contains the Docusaurus site with 4 modules and 4 chapters each, while the backend provides APIs for RAG functionality, content management, and user authentication.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
