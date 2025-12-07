# SpecKit Plus Artifacts Contract: AI Agent <-> SpecKit Plus

**Branch**: `001-physical-ai-book` | **Date**: 2025-12-05 | **Spec**: specs/001-physical-ai-book/spec.md
**Consumer**: AI Agent (Claude Code CLI) | **Provider**: SpecKit Plus Framework

## Overview

This contract defines the expected interaction patterns and standards for AI agents when reading, generating, and updating SpecKit Plus artifacts. Adherence ensures the integrity of the spec-driven development workflow and proper knowledge capture.

## 1. Artifact Reading and Interpretation

### 1.1. `spec.md` (Feature Specification)

*   **Purpose**: AI agents MUST read `spec.md` as the authoritative source for feature requirements, user stories, acceptance criteria, and constraints.
*   **Extraction**: Agents MUST accurately extract functional requirements (FR-), key entities, user scenarios, and success criteria (SC-) to inform subsequent planning and generation tasks.
*   **Path**: `specs/<feature-name>/spec.md`

### 1.2. `plan.md` (Implementation Plan)

*   **Purpose**: AI agents MUST read `plan.md` to understand the high-level architectural decisions, technical context, project structure, and iterative schedules.
*   **Updates**: Agents MAY update sections of `plan.md` during the planning phase, ensuring consistency with research and design decisions.
*   **Path**: `specs/<feature-name>/plan.md`

### 1.3. `tasks.md` (Testable Tasks)

*   **Purpose**: AI agents WILL generate and update `tasks.md` to break down the implementation plan into granular, testable tasks.
*   **Structure**: Each task MUST include a clear description, acceptance criteria, and references to relevant code.
*   **Path**: `specs/<feature-name>/tasks.md`

### 1.4. `.specify/memory/constitution.md` (Project Principles)

*   **Purpose**: AI agents MUST read the project constitution for core principles, development guidelines, and quality standards (e.g., Spec-Driven Development, Modular Content, Naming Conventions).
*   **Adherence**: All AI-generated content and actions MUST comply with the principles outlined in the constitution.
*   **Path**: `.specify/memory/constitution.md`

### 1.5. Prompt History Records (PHRs)

*   **Purpose**: AI agents MUST generate a PHR after every user interaction to capture the prompt, response, and context.
*   **Format**: PHRs MUST follow the specified template (e.g., `.specify/templates/phr-template.prompt.md`) and be accurately filled with metadata.
*   **Routing**: PHRs MUST be routed to the correct subdirectory (`history/prompts/constitution/`, `history/prompts/<feature-name>/`, or `history/prompts/general/`).
*   **Path**: `history/prompts/<category>/<ID>-<slug>.<stage>.prompt.md`

### 1.6. Architectural Decision Records (ADRs)

*   **Purpose**: AI agents WILL suggest the creation of ADRs for architecturally significant decisions.
*   **Trigger**: Suggestions MUST follow the three-part test (Impact, Alternatives, Scope) and require explicit user consent.
*   **Path**: `history/adr/<ID>-<slug>.adr.md`

## 2. Artifact Generation and Updates

### 2.1. Tooling

*   **Preference**: AI agents MUST primarily use agent-native file tools (`Read`, `Write`, `Edit`) for interacting with SpecKit Plus artifacts.
*   **Fallback**: If agent-native tools are unavailable or fail for specific operations (e.g., complex template filling), Bash scripts (e.g., `.specify/scripts/bash/create-phr.sh`) MAY be used as a fallback, followed by agent-native patching to ensure full placeholder resolution.

### 2.2. Placeholders

*   **Resolution**: AI agents MUST ensure that all placeholders (e.g., `{{FIELD_NAME}}`, `[PLACEHOLDER]`) in templates are accurately resolved before writing/updating artifacts. No unresolved placeholders are permitted in final artifacts.

### 2.3. Incremental Updates

*   **Strategy**: When updating existing artifacts (e.g., `plan.md`), AI agents SHOULD aim for minimal, targeted edits to preserve existing content and structure unless a full regeneration is explicitly required.

## 3. Validation and Reporting

*   **Consistency Checks**: AI agents SHOULD perform internal consistency checks between related artifacts (e.g., `plan.md` aligning with `spec.md`).
*   **Error Reporting**: Any failures in reading, generating, or updating SpecKit Plus artifacts MUST be reported, indicating the specific issue and affected file.
