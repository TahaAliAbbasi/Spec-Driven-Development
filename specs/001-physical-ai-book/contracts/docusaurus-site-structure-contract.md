# Docusaurus Site Structure Contract: AI Agent <-> Docusaurus Folder Structure

**Branch**: `001-physical-ai-book` | **Date**: 2025-12-05 | **Spec**: specs/001-physical-ai-book/spec.md
**Consumer**: AI Agent (Claude Code CLI) | **Provider**: Docusaurus Framework

## Overview

This contract defines the expected folder structure and content placement rules for AI agents interacting with the Docusaurus site. Adherence ensures that generated content is correctly integrated, discoverable, and compatible with the Docusaurus build and navigation systems.

## 1. Core Docusaurus Directories

### 1.1. `docs/`

*   **Purpose**: This directory MUST be the primary location for all book content, organized into modules and chapters.
*   **Structure**:
    *   `docs/moduleN-module-title/`: Top-level directories for each module (e.g., `docs/module1-ros2-fundamentals/`).
    *   `docs/moduleN-module-title/chapterX-chapter-title.md`: Markdown files for individual chapters.
    *   `docs/moduleN-module-title/assets/`: Optional subdirectory for module-specific images or static files.
*   **Content Type**: Primarily Markdown (`.md`, `.mdx`).

### 1.2. `src/pages/`

*   **Purpose**: This directory MAY contain standalone React pages (e.g., About, Contributors, custom landing pages) that are not part of the core book documentation flow.
*   **Content Type**: React components (`.js`, `.jsx`, `.ts`, `.tsx`).
*   **AI Agent Interaction**: AI agents SHOULD NOT proactively create or modify files in this directory unless explicitly instructed for specific page development.

### 1.3. `static/`

*   **Purpose**: This directory MUST contain global static assets (images, PDFs, favicons) that are not specific to any single module or chapter, or that need to be directly accessible from the root URL.
*   **Referencing**: Assets placed here are accessible at the site root (e.g., `static/img/logo.png` becomes `/img/logo.png`).
*   **AI Agent Interaction**: AI agents MAY place general-purpose images, diagrams, or downloadable resources here.

### 1.4. `.github/`

*   **Purpose**: Contains GitHub-specific configurations (workflows, issue templates, pull request templates).
*   **AI Agent Interaction**: AI agents MAY interact with these files (e.g., updating issue templates) if within the scope of a GitHub-related task, but SHOULD NOT generate book content here.

### 1.5. `.specify/`

*   **Purpose**: Contains SpecKit Plus templates and scripts for spec-driven development.
*   **AI Agent Interaction**: AI agents WILL read and utilize files from this directory (e.g., `constitution.md`, command templates) to drive their workflow. AI agents SHOULD NOT place generated book content here.

### 1.6. `history/`

*   **Purpose**: Contains Prompt History Records (PHRs) and Architectural Decision Records (ADRs).
*   **AI Agent Interaction**: AI agents WILL generate PHRs and MAY suggest/generate ADRs in this directory. AI agents SHOULD NOT place generated book content here.

## 2. Docusaurus Configuration (`docusaurus.config.js`)

*   **Purpose**: This file configures the entire Docusaurus site, including plugins, themes, navbar, and sidebar.
*   **Path**: Repository root (`docusaurus.config.js`).
*   **AI Agent Interaction**: AI agents MAY need to read this file to understand site navigation or available plugins. AI agents SHOULD NOT directly modify this file unless explicitly instructed for site configuration tasks.

## 3. Sidebar Configuration (`sidebars.js`)

*   **Purpose**: This file defines the structure of the documentation sidebar, mapping `docs/` content to navigation items.
*   **Path**: Repository root (`sidebars.js`).
*   **AI Agent Interaction**: AI agents SHOULD be aware of `sidebars.js` to understand how generated content will appear in the navigation. AI agents SHOULD NOT directly modify this file unless explicitly instructed to update navigation. Content generation should inherently support the sidebar structure through correct frontmatter (`sidebar_position`).

## 4. Content Placement Rules

*   **Modularity**: All book-related Markdown content MUST be placed under `docs/` within its respective module directory.
*   **Assets**: Images directly embedded in Markdown (e.g., diagrams for a specific chapter) SHOULD be placed in a local `assets/` subdirectory within the chapter's folder or the module's `assets/` folder. Global or frequently reused images SHOULD go into `static/img/`.
*   **Consistency**: AI agents MUST maintain a consistent naming convention (`kebab-case`) for files and directories across the documentation structure.
