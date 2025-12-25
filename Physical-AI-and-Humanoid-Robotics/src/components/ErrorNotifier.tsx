// Physical-AI-and-Humanoid-Robotics/src/components/ErrorNotifier.js
// Error handling component

import React, { useEffect, useRef } from 'react';

const ErrorNotifier = ({ message, type = 'error', visible = true, onClose }) => {
  const notifierRef = useRef(null);

  useEffect(() => {
    if (visible && notifierRef.current) {
      // Focus the notifier for screen readers when it appears
      notifierRef.current.focus();
    }
  }, [visible]);

  if (!visible || !message) {
    return null;
  }

  const getTypeClass = (type) => {
    switch (type) {
      case 'success':
        return 'notifier-success';
      case 'warning':
        return 'notifier-warning';
      case 'info':
        return 'notifier-info';
      case 'error':
      default:
        return 'notifier-error';
    }
  };

  const getTypeIcon = (type) => {
    switch (type) {
      case 'success':
        return '✅';
      case 'warning':
        return '⚠️';
      case 'info':
        return 'ℹ️';
      case 'error':
      default:
        return '❌';
    }
  };

  // Determine appropriate ARIA role based on type
  const getAriaRole = (type) => {
    switch (type) {
      case 'error':
        return 'alert';
      case 'warning':
        return 'alert';
      case 'info':
        return 'status';
      case 'success':
        return 'status';
      default:
        return 'status';
    }
  };

  return (
    <div
      ref={notifierRef}
      className={`error-notifier ${getTypeClass(type)}`}
      role={getAriaRole(type)}
      aria-live="polite"
      tabIndex={-1}
    >
      <div className="notifier-content">
        <span className="notifier-icon" aria-hidden="true">{getTypeIcon(type)}</span>
        <span className="notifier-message">{message}</span>
        {onClose && (
          <button
            className="notifier-close"
            onClick={onClose}
            aria-label="Close notification"
            type="button"
          >
            ×
          </button>
        )}
      </div>
    </div>
  );
};

export default ErrorNotifier;