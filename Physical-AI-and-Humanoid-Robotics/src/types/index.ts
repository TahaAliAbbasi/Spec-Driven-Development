// Physical-AI-and-Humanoid-Robotics/src/types/index.ts
// Type definitions for data entities (TypeScript)

/**
 * Validation rules for a query
 */
export interface ValidationRules {
  mustNotBeEmpty: boolean;
  maxCharacters: number;
  xssPrevention: boolean;
}

/**
 * User query input
 */
export interface QueryInput {
  queryText: string;
  queryType: 'global' | 'selected_text_only' | string;
  validationRules: ValidationRules;
}

/**
 * Citation reference
 */
export interface Citation {
  chunkId: string;
  sourceUrl: string;
  textPreview?: string;
}

/**
 * Metadata attached to agent output
 */
export interface AgentMetadata {
  processing_time: number; // milliseconds
  confidence: number; // 0–1
}

/**
 * Agent / AI response output
 */
export interface AgentOutput {
  status: 'ANSWERED' | 'INSUFFICIENT_CONTEXT' | 'REFUSED';
  answer: string;
  citations: Citation[];
  metadata: AgentMetadata;
}

/**
 * User feedback message
 */
export interface UserFeedback {
  type: 'SUCCESS' | 'ERROR' | 'WARNING' | 'INFO';
  message: string;
  timestamp: string;
  relatedAction: string;
}

/**
 * Generic API response
 */
export interface ApiResponse {
  status: string;
  answer: string;
  citations: Record<string, unknown>[];
  metadata: Record<string, unknown>;
}

/**
 * Query context from the UI or document
 */
export interface QueryContext {
  selectedText?: string;
  documentContext?: string[];
}
