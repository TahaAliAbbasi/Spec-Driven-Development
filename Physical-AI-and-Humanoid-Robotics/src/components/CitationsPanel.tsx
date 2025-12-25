// Physical-AI-and-Humanoid-Robotics/src/components/CitationsPanel.tsx
// Citations display component

import React from 'react';

/**
 * Citation object (supports both API and UI formats)
 */
export interface Citation {
  chunkId?: string;
  chunk_id?: string;
  sourceUrl?: string;
  source_url?: string;
  textPreview?: string;
}

/**
 * Props for CitationsPanel
 */
interface CitationsPanelProps {
  citations?: Citation[];
  visible?: boolean;
}

const CitationsPanel: React.FC<CitationsPanelProps> = ({
  citations = [],
  visible = true
}) => {
  if (!visible || citations.length === 0) {
    return (
      <div
        className="citations-panel hidden"
        role="region"
        aria-labelledby="citations-title"
      >
        <h3 id="citations-title">Citations</h3>
        <p
          className="no-citations"
          aria-label="No citations available for this response"
        >
          No citations available for this response.
        </p>
      </div>
    );
  }

  return (
    <div
      className="citations-panel"
      role="region"
      aria-labelledby="citations-title"
      aria-live="polite"
    >
      <h3 id="citations-title">Citations</h3>

      <div className="citations-list" aria-label="List of citations">
        {citations.map((citation, index) => {
          const chunkId = citation.chunk_id ?? citation.chunkId;
          const sourceUrl = citation.source_url ?? citation.sourceUrl;

          return (
            <div key={index} className="citation-item" role="listitem">
              <div className="citation-info">
                {chunkId && (
                  <div
                    className="citation-chunk-id"
                    aria-label={`Chunk ID: ${chunkId}`}
                  >
                    <strong>Chunk ID:</strong> {chunkId}
                  </div>
                )}

                {sourceUrl && (
                  <div
                    className="citation-source"
                    aria-label={`Source: ${sourceUrl}`}
                  >
                    <strong>Source:</strong>{' '}
                    <a
                      href={sourceUrl}
                      target="_blank"
                      rel="noopener noreferrer"
                      className="citation-link"
                      aria-label={`Link to source: ${sourceUrl}`}
                    >
                      {sourceUrl}
                    </a>
                  </div>
                )}

                {citation.textPreview && (
                  <div
                    className="citation-preview"
                    aria-label={`Preview: ${citation.textPreview}`}
                  >
                    <strong>Preview:</strong> {citation.textPreview}
                  </div>
                )}
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );
};

export default CitationsPanel;
