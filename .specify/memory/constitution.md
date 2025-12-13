<!-- Sync Impact Report -->
<!-- Version change: 1.0.0 → 2.0.0 -->
<!-- Modified principles: Complete rewrite to address AI/Spec-Driven Book requirements -->
<!-- Added sections: RAG Rules, Authentication & UX Rules, Technical Enforcement, Extensibility & Evaluation -->
<!-- Removed sections: Previous content structure -->
<!-- Templates requiring updates: .specify/templates/plan-template.md ✅ updated, .specify/templates/spec-template.md ✅ updated, .specify/templates/tasks-template.md ✅ updated, .specify/templates/commands/*.md ✅ updated -->
<!-- Follow-up TODOs: None -->
# Global Constitution for AI-Driven Book Project with RAG Chatbot

## Core Principles

### I. Spec-Driven Development Enforcement
All features, content, and functionality MUST originate from clear, machine-readable specifications using Spec-Kit Plus. No development proceeds without a validated spec.

### II. Zero Hallucination Policy
The RAG chatbot MUST ONLY respond based on provided book content or selected text. Under NO circumstances may the system fabricate, infer, or hallucinate information beyond the provided context.

### III. Deterministic, Explainable AI Behavior
All AI-generated content and responses MUST be reproducible and explainable. The system MUST provide clear attribution to source material when answering questions.

## Agent Governance

### IV. Primary Agent Definition
Claude Code serves as the primary AI author and engineer for all content creation, development, and maintenance tasks.

### V. Subagent Creation Rules
Subagents MAY be created only when they serve reusable intelligence functions. Each subagent MUST have a clear, documented purpose and lifecycle management plan.

### VI. Separation of Concerns
- Content agents: Responsible for book content creation and maintenance
- RAG agents: Handle vector storage, retrieval, and context processing
- Frontend agents: Manage user interface and experience
- Backend agents: Handle API, authentication, and data management

## Content Rules

### VII. Writing Standards
All book content MUST be beginner-friendly while maintaining technical accuracy. Content MUST follow progressive complexity patterns.

### VIII. Consistency Requirements
Chapters MUST maintain consistent terminology, style, and formatting across the entire book. Cross-references MUST be maintained and accurate.

## RAG Rules

### IX. Chunking Strategy
Content MUST be chunked semantically, preserving meaning within each segment. Chunks SHOULD not exceed 1000 tokens and MUST not break logical content units.

### X. Embedding Rules
All content embeddings MUST use the same model and methodology for consistency. Metadata MUST be preserved during embedding processes.

### XI. Context Filtering
The RAG system MUST filter retrieved context to ensure relevance to user queries. Context MUST be validated before generating responses.

### XII. Selected-Text-Only Answering
When users select specific text, the system MUST ONLY respond based on that selected text and related context from the book.

### XIII. Fallback Responses
When no relevant context exists for a query, the system MUST clearly indicate that it cannot answer based on the provided material rather than attempting to answer.

## Authentication & UX Rules

### XIV. Authentication Boundaries
Authentication MUST be implemented using Better Auth. All personalized features MUST be accessible only to authenticated users.

### XV. Personalization Behavior
Personalization MUST adapt chapter content based on user's software/hardware background. Personalization MUST be triggered by explicit user action (button press) and NOT applied silently.

### XVI. Translation Behavior
Urdu translation MUST preserve technical accuracy and meaning. Translation MUST be triggered by explicit user action and be available only to authenticated users.

### XVII. Button-Triggered Transformations
All content modifications (personalization, translation) MUST be initiated by user interaction with specific buttons at the start of each chapter.

### XVIII. No Silent Content Mutation
The system MUST NOT automatically modify content without explicit user consent. All transformations MUST be reversible and transparent.

## Technical Enforcement

### XIX. Mandatory Tech Stack Usage
- Frontend: Docusaurus
- Backend: FastAPI
- Database: Neon Serverless Postgres
- Vector DB: Qdrant Cloud Free Tier
- Authentication: Better Auth
- AI Integration: OpenAI Agents / ChatKit SDKs

### XX. Forbidden Violations
- Direct database manipulation without proper API endpoints
- Bypassing RAG system for chatbot responses
- Storing user data without proper consent and encryption
- Using external APIs without proper integration

### XXI. Deployment Guarantees
The system MUST deploy to GitHub Pages with all functionality intact. The RAG chatbot MUST be embedded within the published book and fully operational.

## Extensibility & Evaluation

### XXII. New Feature Addition Process
New features MUST align with constitutional principles. Features MUST be spec'd using Spec-Kit Plus before implementation begins.

### XXIII. Bonus Point Feature Alignment
- Reusable intelligence via subagents and skills: +50 points
- Better Auth signup/signin: +50 points
- Content personalization per chapter: +50 points
- Urdu translation per chapter: +50 points

### XXIV. Future Phase Compliance
All future project phases MUST comply with this constitution. Deviations REQUIRE formal constitutional amendments.

## Governance
- **Amendment Procedure:** Amendments to this constitution MUST be proposed via PRs, undergo review, and require unanimous approval from core contributors.
- **Versioning Policy:** This constitution follows semantic versioning. Major changes require version increment.
- **Compliance Review:** Regular reviews (at least quarterly) MUST be conducted to ensure ongoing adherence and relevance of these principles.
- **Supersedes:** This constitution supersedes all other informal practices or guidelines.

**Version**: 2.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-13
