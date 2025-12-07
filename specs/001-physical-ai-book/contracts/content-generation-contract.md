# Content Generation Contract: AI Agent <-> Docusaurus Markdown

**Branch**: `001-physical-ai-book` | **Date**: 2025-12-05 | **Spec**: specs/001-physical-ai-book/spec.md
**Consumer**: AI Agent (Claude Code CLI) | **Provider**: Docusaurus Content Structure

## Overview

This contract defines the expected format, standards, and interaction patterns for AI agents when generating and updating content within the Docusaurus book project. Adherence ensures consistency, maintainability, and compatibility with the Docusaurus build process.

## 1. Content Format and Standards (FR-008)

### 1.1. Markdown Syntax

*   **Standard**: All content MUST be generated using GitHub-Flavored Markdown (GFM).
*   **Extensions**: MDX (Markdown with JSX) MAY be used for embedding React components (e.g., interactive diagrams, custom warnings), but basic content should remain pure GFM.
*   **Code Blocks**: Code examples MUST use fenced code blocks with language highlighting specified (e.g., ` ```python `).
*   **Headings**: Page titles MUST be H1 (`#`), followed by H2 (`##`), H3 (`###`), etc. for sub-sections.
*   **Lists**: Ordered and unordered lists MUST be correctly formatted.

### 1.2. Images and Media

*   **Location**: Images and other static assets MUST be placed in `static/` or module-specific `docs/moduleN/assets/` directories.
*   **Referencing**: Images MUST be referenced using relative paths from the markdown file or absolute paths from the site root (e.g., `![Alt Text](/img/my-image.png)` or `![Alt Text](./assets/chapter-diagram.png)`).

### 1.3. Internal and External Links

*   **Internal Links**: MUST use Docusaurus's internal linking syntax (e.g., `<Link to="/docs/module1/chapter1">`).
*   **External Links**: MUST use standard Markdown link syntax (e.g., `[Anthropic](https://www.anthropic.com)`).

### 1.4. Metadata (Frontmatter)

*   **Requirement**: Each Markdown file MUST include Docusaurus-compatible YAML frontmatter at the top.
*   **Mandatory Fields**:
    *   `title`: The title of the page/chapter.
    *   `sidebar_position`: Numeric order within its sidebar category.
*   **Optional Fields**:
    *   `description`: Short summary for SEO/metadata.
    *   `keywords`: Relevant keywords for search.
    *   `slug`: Custom URL path.

## 2. Iterative Content Generation Workflow (FR-010)

### 2.1. Initial Draft Generation

*   **Input**: Detailed module/chapter specifications (`spec.md`).
*   **Output**: Initial Markdown file drafts (`docs/moduleN/chapterX.md`) with ~70% completeness.
*   **Content**: Textual explanations, conceptual diagrams (as `TODO` placeholders or initial Mermaid/Graphviz code), and basic code examples.

### 2.2. Targeted Refinement

*   **Input**: Human feedback on existing Markdown content (e.g., specific edits, areas for expansion, factual corrections).
*   **Process**: AI agent MUST read the existing file, understand the feedback, and apply targeted edits to improve accuracy, clarity, and completeness (~90% goal).
*   **Tooling**: Claude Code CLI `Edit` or `Write` tools will be used, ensuring precise string replacement or full file overwrite as appropriate.

### 2.3. Code Example Verification

*   **Requirement**: All code examples generated or modified by the AI MUST be syntactically correct and, where applicable, executable.
*   **Integration**: AI agents MAY suggest or generate unit tests or verification scripts for complex code examples.

## 3. Naming Conventions & Folder Structure (FR-007)

### 3.1. File Naming

*   **Markdown Files**: MUST be `kebab-case.md` (e.g., `ros2-fundamentals.md`, `chapter-nodes-topics.md`).
*   **Asset Files**: MUST be `kebab-case.png`, `kebab-case.jpg`, etc.

### 3.2. Folder Structure

*   **Modules**: `docs/moduleN/` (e.g., `docs/module1-ros2/`).
*   **Chapters**: `docs/moduleN/chapterX/` (optional, for larger chapters with sub-assets).
*   **Assets**: `docs/moduleN/assets/` or `static/`.

## 4. Error Handling & Feedback

*   **Invalid Generation**: If an AI agent cannot generate content conforming to this contract, it MUST report the issue, explaining the specific constraint violated.
*   **Human Review Integration**: AI agents should anticipate iterative feedback from human reviewers and be prepared to make revisions based on specific instructions.
