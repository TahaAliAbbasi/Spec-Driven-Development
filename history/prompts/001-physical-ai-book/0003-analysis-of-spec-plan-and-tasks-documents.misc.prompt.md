---
id: 0003
title: Analysis of Spec, Plan, and Tasks Documents
stage: misc
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-book
branch: 001-physical-ai-book
user: user
command: /sp.analyze
labels: ["analysis", "quality-check"]
links:
  spec: specs/001-physical-ai-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-book/spec.md
  - specs/001-physical-ai-book/plan.md
  - specs/001-physical-ai-book/tasks.md
tests:

---

## Prompt

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Goal

Identify inconsistencies, duplications, ambiguities, and underspecified items across the three core artifacts (`spec.md`, `plan.md`, `tasks.md`) before implementation. This command MUST run only after `/sp.tasks` has successfully produced a complete `tasks.md`.

## Operating Constraints

**STRICTLY READ-ONLY**: Do **not** modify any files. Output a structured analysis report. Offer an optional remediation plan (user must explicitly approve before any follow-up editing commands would be invoked manually).

**Constitution Authority**: The project constitution (`.specify/memory/constitution.md`) is **non-negotiable** within this analysis scope. Constitution conflicts are automatically CRITICAL and require adjustment of the spec, plan, or tasks—not dilution, reinterpretation, or silent ignoring of the principle. If a principle itself needs to change, that must occur in a separate, explicit constitution update outside `/sp.analyze`.

## Execution Steps

### 1. Initialize Analysis Context

Run `.specify/scripts/powershell/check-prerequisites.ps1 -Json -RequireTasks -IncludeTasks` once from repo root and parse JSON for FEATURE_DIR and AVAILABLE_DOCS. Derive absolute paths:

- SPEC = FEATURE_DIR/spec.md
- PLAN = FEATURE_DIR/plan.md
- TASKS = FEATURE_DIR/tasks.md

Abort with an error message if any required file is missing (instruct the user to run missing prerequisite command).
For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

### 2. Load Artifacts (Progressive Disclosure)

Load only the minimal necessary context from each artifact:

**From spec.md:**

- Overview/Context
- Functional Requirements
- Non-Functional Requirements
- User Stories
- Edge Cases (if present)

**From plan.md:**

- Architecture/stack choices
- Data Model references
- Phases
- Technical constraints

**From tasks.md:**

- Task IDs
- Descriptions
- Phase grouping
- Parallel markers [P]
- Referenced file paths

**From constitution:**

- Load `.specify/memory/constitution.md` for principle validation

### 3. Build Semantic Models

Create internal representations (do not include raw artifacts in output):

- **Requirements inventory**: Each functional + non-functional requirement with a stable key (derive slug based on imperative phrase; e.g., "User can upload file" → `user-can-upload-file`)
- **User story/action inventory**: Discrete user actions with acceptance criteria
- **Task coverage mapping**: Map each task to one or more requirements or stories (inference by keyword / explicit reference patterns like IDs or key phrases)
- **Constitution rule set**: Extract principle names and MUST/SHOULD normative statements

### 4. Detection Passes (Token-Efficient Analysis)

Focus on high-signal findings. Limit to 50 findings total; aggregate remainder in overflow summary.

#### A. Duplication Detection

- Identify near-duplicate requirements
- Mark lower-quality phrasing for consolidation

#### B. Ambiguity Detection

- Flag vague adjectives (fast, scalable, secure, intuitive, robust) lacking measurable criteria
- Flag unresolved placeholders (TODO, TKTK, ???, `<placeholder>`, etc.)

#### C. Underspecification

- Requirements with verbs but missing object or measurable outcome
- User stories missing acceptance criteria alignment
- Tasks referencing files or components not defined in spec/plan

#### D. Constitution Alignment

- Any requirement or plan element conflicting with a MUST principle
- Missing mandated sections or quality gates from constitution

#### E. Coverage Gaps

- Requirements with zero associated tasks
- Tasks with no mapped requirement/story
- Non-functional requirements not reflected in tasks (e.g., performance, security)

#### F. Inconsistency

- Terminology drift (same concept named differently across files)
- Data entities referenced in plan but absent in spec (or vice versa)
- Task ordering contradictions (e.g., integration tasks before foundational setup tasks without dependency note)
- Conflicting requirements (e.g., one requires Next.js while other specifies Vue)

### 5. Severity Assignment

Use this heuristic to prioritize findings:

- **CRITICAL**: Violates constitution MUST, missing core spec artifact, or requirement with zero coverage that blocks baseline functionality
- **HIGH**: Duplicate or conflicting requirement, ambiguous security/performance attribute, untestable acceptance criterion
- **MEDIUM**: Terminology drift, missing non-functional task coverage, underspecified edge case
- **LOW**: Style/wording improvements, minor redundancy not affecting execution order

### 6. Produce Compact Analysis Report

Output a Markdown report (no file writes) with the following structure:

## Specification Analysis Report

| ID | Category | Severity | Location(s) | Summary | Recommendation |
|----|----------|----------|-------------|---------|----------------|
| A1 | Duplication | HIGH | spec.md:L120-134 | Two similar requirements ... | Merge phrasing; keep clearer version |

(Add one row per finding; generate stable IDs prefixed by category initial.)

**Coverage Summary Table:**

| Requirement Key | Has Task? | Task IDs | Notes |
|-----------------|-----------|----------|-------|

**Constitution Alignment Issues:** (if any)

**Unmapped Tasks:** (if any)

**Metrics:**

- Total Requirements
- Total Tasks
- Coverage % (requirements with >=1 task)
- Ambiguity Count
- Duplication Count
- Critical Issues Count

### 7. Provide Next Actions

At end of report, output a concise Next Actions block:

- If CRITICAL issues exist: Recommend resolving before `/sp.implement`
- If only LOW/MEDIUM: User may proceed, but provide improvement suggestions
- Provide explicit command suggestions: e.g., "Run /sp.specify with refinement", "Run /sp.plan to adjust architecture", "Manually edit tasks.md to add coverage for 'performance-metrics'"

### 8. Offer Remediation

Ask the user: "Would you like me to suggest concrete remediation edits for the top N issues?" (Do NOT apply them automatically.)

## Operating Principles

### Context Efficiency

- **Minimal high-signal tokens**: Focus on actionable findings, not exhaustive documentation
- **Progressive disclosure**: Load artifacts incrementally; don't dump all content into analysis
- **Token-efficient output**: Limit findings table to 50 rows; summarize overflow
- **Deterministic results**: Rerunning without changes should produce consistent IDs and counts

### Analysis Guidelines

- **NEVER modify files** (this is read-only analysis)
- **NEVER hallucinate missing sections** (if absent, report them accurately)
- **Prioritize constitution violations** (these are always CRITICAL)
- **Use examples over exhaustive rules** (cite specific instances, not generic patterns)
- **Report zero issues gracefully** (emit success report with coverage statistics)

## Context

$ARGUMENTS

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.

## Response snapshot

## Specification Analysis Report

| ID | Category | Severity | Location(s) | Summary | Recommendation |
|----|----------|----------|-------------|---------|----------------|
| R1 | Inconsistency | MEDIUM | spec.md:70, plan.md:71 | The spec mentions "Capstone Project: Humanoid Robot Control (Priority: P1)" for User Story 5, while the plan lists "Capstone Project: The Autonomous Humanoid". Slight terminology difference. | Align the title of the Capstone Project in `plan.md` to match `spec.md` for consistency. |
| R2 | Ambiguity | LOW | spec.md:86-90 | Edge cases are listed but lack specific mitigation strategies or how the system should handle them. | Consider adding high-level mitigation strategies or a plan to address these edge cases in future planning. |
| R3 | Coverage Gap | MEDIUM | plan.md:14-15 | The plan mentions automated linting/formatting for code examples and human content review, but `tasks.md` only has `T012` for linting/formatting. Human content review is not explicitly tasked. | Add tasks for explicit human review of content in `tasks.md` to align with the plan's testing strategy. |
| R4 | Underspecification | MEDIUM | plan.md:42 | `quickstart.md` is mentioned as "to be created" in the plan, but `tasks.md` has `T043` for "Validate quickstart.md for accuracy and completeness" which implies it should exist or be created earlier. | Add a task in Phase 1 or 2 of `tasks.md` to explicitly create the `quickstart.md` file before it's validated in Phase 8. |
| R5 | Underspecification | MEDIUM | spec.md:95 | The clarification about learning outcomes being at both module and chapter levels is not explicitly reflected in distinct tasks for generating/defining these. | Consider adding specific tasks in `tasks.md` for defining learning outcomes at both module and chapter levels for each content generation task. |
| R6 | Constitution Alignment | MEDIUM | constitution.md:40, plan.md:14-15 | Constitution states diagrams and illustrations SHOULD be provided for complex concepts. Plan mentions human content review, but tasks don't explicitly call for generating or reviewing diagrams. | Add tasks to `tasks.md` for generating, reviewing, and integrating diagrams or illustrations where appropriate in the content modules. |

**Coverage Summary Table:**

| Requirement Key | Has Task? | Task IDs | Notes |
|-----------------|-----------|----------|-------|
| FR-001 | Yes | T013, T014, T015, T018, T019, T020, T024, T025, T026, T030, T031, T034, T035 | Covered across multiple content generation tasks. |
| FR-002 | Yes | T016, T021, T022, T027, T028, T032 | Covered by tasks for exercises, code examples, and lab guides. |
| FR-003 | Yes | T009 | Covered by documenting hardware requirements. |
| FR-004 | Yes | T009, T010 | Covered by documenting lab setup procedures. |
| FR-005 | No | | Learning outcomes are mentioned, but no explicit tasks to *define* them. |
| FR-006 | Yes | T016 | Covered by tasks for creating assessments. |
| FR-007 | Yes | T003, T004, T005, T006, T007, T008, T017, T023, T029, T033, T037, T041 | Covered by tasks for creating directory structure and content review. |
| FR-008 | Yes | T017, T023, T029, T033, T037, T041 | Covered by tasks for content review and consistent formatting. |
| FR-009 | Yes | T007 | Implicitly covered by using SpecKit Plus for task generation. |
| FR-010 | Yes | T013, T014, T015, T018, T019, T020, T024, T025, T026, T030, T031, T034, T035 | Covered by AI content generation tasks. |
| FR-011 | No | | High-performance simulation requirement is not explicitly tasked for implementation or verification. |
| FR-012 | No | | Guidance for physical AI lab or cloud fallback setups is not explicitly tasked for documentation or setup. |
| FR-013 | Yes | T003 | Covered by creating modular `docs/` structure. |
| SC-001 | No | | Capstone project completion rate is a metric, not a task for achievement within tasks.md. |
| SC-002 | No | | Pass mark on module assessments is a metric, not a task for achievement within tasks.md. |
| SC-003 | No | | Human review pass rate for AI-generated content is a metric, not a task for achievement within tasks.md. |
| SC-004 | Yes | T042 | Covered by task for Docusaurus build verification. |
| SC-005 | Yes | T038 | Covered by task to update CLAUDE.md, and implicitly by versioning in plan.md. |
| SC-006 | No | | Simulation FPS is a performance goal, not a task for achievement within tasks.md. |

**Constitution Alignment Issues:**

- **Documentation Standards (Constitution.md:40)**: "Diagrams and illustrations SHOULD be provided for complex concepts." This is not explicitly tasked in `tasks.md`. (Severity: MEDIUM)

**Unmapped Tasks:**

- None

**Metrics:**

- Total Requirements: 19 (13 FR, 6 SC)
- Total Tasks: 43
- Coverage % (requirements with >=1 task): 68.4% (13 out of 19 requirements have at least one task)
- Ambiguity Count: 1
- Duplication Count: 0
- Critical Issues Count: 0

## Next Actions

- **HIGH/MEDIUM Issues**:
    - **R1 (Inconsistency)**: Edit `plan.md` to align the Capstone Project title with `spec.md`.
    - **R3 (Coverage Gap)**: Edit `tasks.md` to add explicit tasks for human content review in each module's review phase.
    - **R4 (Underspecification)**: Edit `tasks.md` to add a task in an earlier phase for creating `quickstart.md`.
    - **R5 (Underspecification)**: Edit `tasks.md` to add specific tasks for defining learning outcomes at both module and chapter levels.
    - **R6 (Constitution Alignment)**: Edit `tasks.md` to add tasks for generating and reviewing diagrams/illustrations.
- **Coverage Gaps**:
    - Consider adding tasks to address FR-005 (define learning outcomes), FR-011 (high-performance simulation requirements), FR-012 (physical AI lab or cloud fallback setups), and SC-001, SC-002, SC-003, SC-006 (measurable outcomes). While SCs are metrics, tasks should contribute to their achievement.

