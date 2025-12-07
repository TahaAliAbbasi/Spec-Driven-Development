<!-- Sync Impact Report -->
<!-- Version change: 0.0.0 → 1.0.0 -->
<!-- Modified principles: All new -->
<!-- Added sections: Folder Structure & Documentation Standards, Contribution & Quality Workflow -->
<!-- Removed sections: None -->
<!-- Templates requiring updates: .specify/templates/plan-template.md ⚠ pending, .specify/templates/spec-template.md ⚠ pending, .specify/templates/tasks-template.md ⚠ pending, .specify/templates/commands/*.md ⚠ pending -->
<!-- Follow-up TODOs: None -->
# Physical AI & Humanoid Robotics Book Project Constitution

## Core Principles

### I. Spec-Driven Development
All features and content MUST originate from a clear, concise, and machine-readable specification.

### II. Modular Content
Each book section, chapter, or major topic MUST be treated as a modular unit, independently developable, testable, and documentable.

### III. Docusaurus Best Practices
Adhere to Docusaurus configuration guidelines, theme customization, and content structuring for optimal performance and maintainability.

### IV. AI-Assisted Content Generation
AI agents MUST be instructed with clear prompts, provide content that adheres to factual accuracy and style guides, and facilitate iterative refinement.

### V. Clear Naming Conventions
Standardize file, folder, variable, and component naming to enhance readability, searchability, and maintainability.

### VI. Versioned Releases
Content updates and major revisions MUST follow semantic versioning, with clear release notes for each published version.

## Folder Structure & Documentation Standards
- **Folder Structure:**
    - `docs/`: Markdown files for book content. Structured hierarchically by chapters/sections.
    - `src/pages/`: Additional standalone pages (e.g., About, Contributors).
    - `static/`: Static assets (images, PDFs).
    - `.github/`: GitHub-specific configs (workflows, issue templates).
    - `.specify/`: SpecKit Plus templates and scripts.
    - `history/`: Prompt History Records (PHRs) and Architectural Decision Records (ADRs).
- **Documentation Standards:**
    - All content MUST be written in GitHub-flavored Markdown.
    - Code examples MUST be accurate, tested, and properly formatted with language highlighting.
    - Diagrams and illustrations SHOULD be provided for complex concepts.
    - External references MUST be cited with appropriate links.

## Contribution & Quality Workflow
- **Contribution Workflow:**
    - All changes MUST be proposed via Pull Requests (PRs).
    - PRs require at least one approving review before merging.
    - Feature branches MUST be used for all development.
    - Commit messages MUST follow Conventional Commits specification.
- **Quality & Testing Guidelines:**
    - Content accuracy MUST be verified (human review + automated checks where possible).
    - Code examples MUST pass automated tests (if applicable).
    - Docusaurus build MUST pass without errors or warnings.
    - Accessibility (a11y) guidelines SHOULD be considered for all new UI components/content.

## Governance
- **Amendment Procedure:** Amendments to this constitution MUST be proposed via PRs, undergo review, and require unanimous approval from core contributors.
- **Versioning Policy:** This constitution itself follows semantic versioning.
- **Compliance Review:** Regular reviews (at least quarterly) MUST be conducted to ensure ongoing adherence and relevance of these principles.
- **Supersedes:** This constitution supersedes all other informal practices or guidelines.

**Version**: 1.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04
