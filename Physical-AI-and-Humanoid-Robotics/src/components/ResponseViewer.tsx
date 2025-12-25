// Physical-AI-and-Humanoid-Robotics/src/components/ResponseViewer.js
// Response display component

import React from 'react';
import CitationsPanel from './CitationsPanel';
import { RESPONSE_STATUS, COMPONENT_DEFAULTS } from '../config/constants';

const ResponseViewer = ({ response, loading, error }) => {
  if (loading) {
    return (
      <div className="response-viewer loading" role="status" aria-live="polite">
        <div className="loading-spinner" aria-hidden="true">⏳</div>
        <p>{COMPONENT_DEFAULTS.LOADING_TEXT}</p>
      </div>
    );
  }

  if (error) {
    return (
      <div className="response-viewer error" role="alert">
        <div className="error-icon" aria-hidden="true">❌</div>
        <p>{error}</p>
      </div>
    );
  }

  if (!response) {
    return (
      <div className="response-viewer empty" aria-label="Response area - waiting for query">
        <p>Submit a query to see the AI-generated response here.</p>
      </div>
    );
  }

  const { status, answer, citations } = response;

  let content;
  let ariaLabel = '';

  switch (status) {
    case RESPONSE_STATUS.ANSWERED:
      content = (
        <div className="response-content answered">
          <div className="response-answer">
            <h3>Response:</h3>
            <p>{answer}</p>
          </div>
          <CitationsPanel citations={citations} visible={true} />
        </div>
      );
      ariaLabel = 'Answered response with citations';
      break;
    case RESPONSE_STATUS.INSUFFICIENT_CONTEXT:
      content = (
        <div className="response-content insufficient-context">
          <div className="response-message">
            <h3>Unable to Answer:</h3>
            <p>{COMPONENT_DEFAULTS.ERROR_MESSAGES.INSUFFICIENT_CONTEXT}</p>
          </div>
          <CitationsPanel citations={citations} visible={true} />
        </div>
      );
      ariaLabel = 'Response indicates insufficient context';
      break;
    case RESPONSE_STATUS.REFUSED:
      content = (
        <div className="response-content refused">
          <div className="response-message">
            <h3>Query Refused:</h3>
            <p>{COMPONENT_DEFAULTS.ERROR_MESSAGES.REFUSED}</p>
          </div>
          <CitationsPanel citations={citations} visible={true} />
        </div>
      );
      ariaLabel = 'Response indicates query was refused';
      break;
    default:
      content = (
        <div className="response-content unknown">
          <div className="response-message">
            <p>Unknown response status: {status}</p>
          </div>
          <CitationsPanel citations={citations} visible={true} />
        </div>
      );
      ariaLabel = 'Response with unknown status';
  }

  return (
    <div className={`response-viewer ${status.toLowerCase()}`} aria-label={ariaLabel}>
      {content}
    </div>
  );
};

export default ResponseViewer;