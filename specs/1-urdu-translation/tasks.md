---
description: "Task list for Urdu Translation functionality implementation"
---

# Tasks: Urdu Translation Functionality

**Input**: Design documents from `/specs/1-urdu-translation/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus Project**: `Physical-AI-and-Humanoid-Robotics/` structure with `docs/`, `src/`, `i18n/` directories

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Docusaurus i18n configuration and basic structure

- [X] T001 Update Docusaurus configuration to enable i18n support in Physical-AI-and-Humanoid-Robotics/docusaurus.config.ts
- [X] T002 [P] Create directory structure for Urdu translations in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/
- [X] T003 [P] Install any required dependencies for RTL support if needed

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core i18n infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Configure Docusaurus to support English and Urdu locales in Physical-AI-and-Humanoid-Robotics/docusaurus.config.ts
- [X] T005 [P] Add localeDropdown to navbar items in Physical-AI-and-Humanoid-Robotics/docusaurus.config.ts
- [X] T006 [P] Implement RTL CSS support in Physical-AI-and-Humanoid-Robotics/src/css/custom.css
- [X] T007 Create initial Urdu translation directory structure to match English content
- [X] T008 Verify Docusaurus build works with new i18n configuration

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Language Switching (Priority: P1) 🎯 MVP

**Goal**: Enable users to switch between English and Urdu languages via a translation button in the navigation bar

**Independent Test**: Can be fully tested by clicking the translation button in the navigation bar and verifying that the content changes from English to Urdu while maintaining the layout and functionality.

### Implementation for User Story 1

- [X] T009 [P] [US1] Add Urdu locale configuration with RTL support in Physical-AI-and-Humanoid-Robotics/docusaurus.config.ts
- [X] T010 [US1] Verify language switcher dropdown appears in navigation bar
- [X] T011 [US1] Create initial Urdu translation for intro.md in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/intro.md
- [X] T012 [US1] Create initial Urdu translation for prerequisites.md in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/prerequisites.md
- [X] T013 [US1] Test language switching between English and Urdu on intro page
- [X] T014 [US1] Verify layout and functionality remain intact when switching languages

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Persistent Language Preference (Priority: P2)

**Goal**: Remember the user's language preference across sessions using browser storage

**Independent Test**: Can be tested by switching to Urdu, closing the browser, reopening, and verifying that the content loads in Urdu by default.

### Implementation for User Story 2

- [X] T015 [P] [US2] Verify Docusaurus automatically stores language preference in localStorage
- [X] T016 [US2] Test that language preference persists across browser sessions
- [X] T017 [US2] Create Urdu translation for interactive-query.md in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/interactive-query.md
- [X] T018 [US2] Add additional content to test persistent language preference
- [X] T019 [US2] Validate language preference remains for at least 30 days

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Visual Language Indicator (Priority: P3)

**Goal**: Provide clear visual indication of the currently selected language

**Independent Test**: Can be tested by verifying that there's a clear indicator showing the currently selected language (either English or Urdu).

### Implementation for User Story 3

- [X] T020 [P] [US3] Verify language switcher shows current language clearly
- [X] T021 [US3] Test visual indication when language is English
- [X] T022 [US3] Test visual indication when language is Urdu
- [X] T023 [US3] Add custom styling to improve language indicator visibility if needed in Physical-AI-and-Humanoid-Robotics/src/css/custom.css

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Content Translation - Module 1 (Priority: P1)

**Goal**: Translate Module 1 content to Urdu to ensure complete functionality across all pages

### Implementation for Module 1 Translation

- [X] T024 [P] Translate module1/README.md to Urdu in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module1/README.md
- [X] T025 [P] Translate module1/chapter1.md to Urdu in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module1/chapter1.md
- [X] T026 [P] Translate module1/chapter2.md to Urdu in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module1/chapter2.md
- [X] T027 [P] Translate module1/chapter3.md to Urdu in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module1/chapter3.md
- [X] T028 Translate module1/exercises.md to Urdu in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module1/exercises.md

---

## Phase 7: Content Translation - Module 2 (Priority: P2)

**Goal**: Translate Module 2 content to Urdu

### Implementation for Module 2 Translation

- [X] T029 [P] Translate module2/README.md to Urdu in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module2/README.md
- [X] T030 [P] Translate module2/chapter1.md to Urdu in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module2/chapter1.md
- [X] T031 [P] Translate module2/chapter2.md to Urdu in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module2/chapter2.md
- [X] T032 [P] Translate module2/chapter3.md to Urdu in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module2/chapter3.md

---

## Phase 8: Content Translation - Module 3 (Priority: P3)

**Goal**: Translate Module 3 content to Urdu

### Implementation for Module 3 Translation

- [X] T033 [P] Translate module3/README.md to Urdu in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module3/README.md
- [X] T034 [P] Translate module3/chapter1.md to Urdu in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module3/chapter1.md
- [X] T035 [P] Translate module3/chapter2.md to Urdu in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module3/chapter2.md
- [X] T036 [P] Translate module3/chapter3.md to Urdu in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module3/chapter3.md

---

## Phase 9: Content Translation - Module 4 (Priority: P4)

**Goal**: Translate Module 4 content to Urdu

### Implementation for Module 4 Translation

- [X] T037 [P] Translate module4/README.md to Urdu in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module4/README.md
- [X] T038 [P] Translate module4/chapter1.md to Urdu in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module4/chapter1.md
- [X] T039 [P] Translate module4/chapter2.md to Urdu in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module4/chapter2.md

---

## Phase 10: Content Translation - Module 5 (Priority: P5)

**Goal**: Translate Module 5 content to Urdu

### Implementation for Module 5 Translation

- [X] T040 [P] Translate module5/README.md to Urdu in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module5/README.md
- [X] T041 [P] Translate module5/guidelines.md to Urdu in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module5/guidelines.md
- [X] T042 [P] Translate module5/sample_solution.md to Urdu in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module5/sample_solution.md
- [X] T043 [P] Translate module5/troubleshooting.md to Urdu in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/module5/troubleshooting.md

---

## Phase 11: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T044 [P] Test all pages in Urdu to ensure proper RTL rendering
- [X] T045 [P] Verify all navigation and links work correctly in Urdu
- [X] T046 [P] Optimize CSS for RTL layout if needed
- [X] T047 [P] Test accessibility features with screen readers in Urdu
- [X] T048 [P] Validate that 95% of web book pages successfully display in Urdu
- [X] T049 [P] Performance testing to ensure language switching is under 2 seconds
- [X] T050 [P] Final validation of all functionality in both languages

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Content Translation Phases (6-10)**: Can start after foundational phase but should follow priority order
- **Polish (Final Phase)**: Depends on all desired content being translated

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Content Translation Dependencies

- **Module translations** can be worked on in parallel after foundational phase
- All module translations depend on the foundational i18n setup

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Different user stories can be worked on in parallel by different team members
- All content translation tasks within modules can run in parallel
- All polish tasks marked [P] can run in parallel

---

## Parallel Example: User Story 1

```bash
# Launch all components for User Story 1 together:
Task: "Add Urdu locale configuration with RTL support in Physical-AI-and-Humanoid-Robotics/docusaurus.config.ts"
Task: "Create initial Urdu translation for intro.md in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/intro.md"
Task: "Create initial Urdu translation for prerequisites.md in Physical-AI-and-Humanoid-Robotics/i18n/ur/docusaurus-plugin-content-docs/current/prerequisites.md"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add Module translations → Test independently → Deploy/Demo
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
   - Developer D: Module 1 translation
   - Developer E: Module 2 translation
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence