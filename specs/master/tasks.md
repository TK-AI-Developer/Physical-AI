---
description: "Task list for the Physical AI Book feature"
---

# Tasks: Physical AI Book

**Input**: Design documents from `/specs/master/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: No tests were explicitly requested, so test tasks are not included.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create project structure for backend and frontend in `backend/` and `frontend/`
- [x] T002 Initialize Python project for the backend with FastAPI in `backend/`
- [x] T003 [P] Initialize Node.js project for the frontend with Docusaurus in `frontend/`
- [x] T004 [P] Configure linting and formatting for the backend in `backend/`
- [x] T005 [P] Configure linting and formatting for the frontend in `frontend/`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

- [x] T006 Setup database schema for SQLite in `backend/src/database.py`
- [x] T007 Setup API routing and middleware structure in `backend/src/main.py`
- [x] T008 [P] Configure error handling and logging infrastructure in `backend/src/`
- [x] T009 [P] Create base models for Chapter and Module in `backend/src/models/`

---

## Phase 3: User Story 1 - View Book Content (Priority: P1) 🎯 MVP

**Goal**: As a user, I want to see the chapters and modules of the book so that I can read the content.

**Independent Test**: The user can navigate to the website and see a list of chapters. Clicking on a chapter shows the modules within it. Clicking on a module displays its content.

### Implementation for User Story 1

- [x] T010 [US1] Implement FastAPI service to read chapter and module data from markdown files in `backend/src/services/content.py`
- [x] T011 [US1] Implement API endpoint for `/chapters` in `backend/src/api/chapters.py`
- [x] T012 [P] [US1] Implement API endpoint for `/chapters/{slug}` in `backend/src/api/chapters.py`
- [ ] T013 [P] [US1] Create React component to display a list of chapters in `frontend/src/components/ChapterList.tsx`
- [ ] T014 [US1] Create React component to display a single chapter with its modules in `frontend/src/components/ChapterView.tsx`
- [ ] T015 [P] [US1] Create React component to display a single module in `frontend/src/components/ModuleView.tsx`
- [ ] T016 [US1] Fetch data from the backend API and display it on the main page `frontend/src/pages/index.tsx`

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T017 [P] Documentation updates in `docs/`
- [ ] T018 Code cleanup and refactoring across the codebase
- [ ] T019 Run quickstart.md validation

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories

### Within Each User Story

- Models before services
- Services before endpoints
- Core implementation before integration

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready
