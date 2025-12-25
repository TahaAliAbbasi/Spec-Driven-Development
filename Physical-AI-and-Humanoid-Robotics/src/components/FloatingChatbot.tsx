// Physical-AI-and-Humanoid-Robotics/src/components/FloatingChatbot.tsx
// Floating chatbot component that appears on every page

import React, { useState, useEffect, useRef } from 'react';
import '../css/floating-chatbot.css';

// Define types directly to avoid import issues
interface ApiResponse {
  status: string;
  answer: string;
  citations: Record<string, unknown>[];
  metadata: Record<string, unknown>;
}

interface Notification {
  message: string;
  type: 'success' | 'error';
}

interface Message {
  id: string;
  text: string;
  sender: 'user' | 'ai';
  timestamp: Date;
}

// Simple fallback components if the main ones fail to import
const FallbackQueryInput: React.FC<{ onSubmit: (query: string) => void; disabled: boolean; placeholder: string }> = ({ onSubmit, disabled, placeholder }) => {
  const [query, setQuery] = useState('');

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (query.trim()) {
      onSubmit(query.trim());
      setQuery('');
    }
  };

  return (
    <form onSubmit={handleSubmit} style={{ display: 'flex', gap: '8px' }}>
      <input
        type="text"
        value={query}
        onChange={(e) => setQuery(e.target.value)}
        placeholder={placeholder}
        disabled={disabled}
        style={{
          flex: 1,
          padding: '12px 16px',
          border: '2px solid #e1e5f2',
          borderRadius: '20px',
          fontSize: '14px',
          outline: 'none'
        }}
      />
      <button
        type="submit"
        disabled={disabled || !query.trim()}
        style={{
          padding: '12px 20px',
          backgroundColor: '#667eea',
          color: 'white',
          border: 'none',
          borderRadius: '20px',
          cursor: disabled || !query.trim() ? 'not-allowed' : 'pointer',
          fontSize: '14px',
          fontWeight: '600'
        }}
      >
        Send
      </button>
    </form>
  );
};

const FallbackErrorNotifier: React.FC<{ message: string; type: string; visible: boolean; onClose: () => void }> = ({ message, type, visible, onClose }) => {
  if (!visible) return null;
  return (
    <div style={{
      padding: '12px',
      margin: '8px 0',
      backgroundColor: type === 'error' ? '#ffebee' : '#e8f5e8',
      border: `1px solid ${type === 'error' ? '#ffcdd2' : '#c8e6c8'}`,
      borderRadius: '12px'
    }}>
      {message}
      <button onClick={onClose} style={{ marginLeft: '8px', background: 'none', border: 'none', cursor: 'pointer' }}>×</button>
    </div>
  );
};

const FloatingChatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState<boolean>(false);
  const [loading, setLoading] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);
  const [notification, setNotification] = useState<Notification | null>(null);
  const [selectedText, setSelectedText] = useState<string>('');
  const [showTextSelectionButton, setShowTextSelectionButton] = useState<boolean>(false);
  const [textSelectionPosition, setTextSelectionPosition] = useState<{ x: number; y: number }>({ x: 0, y: 0 });
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState<string>('');

  const textSelectionTimeout = useRef<NodeJS.Timeout | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Add initial greeting message
  useEffect(() => {
    if (isOpen && messages.length === 0) {
      setMessages([
        {
          id: 'greeting',
          text: 'Hello! How may I help you with the Physical AI and Humanoid Robotics book?',
          sender: 'ai',
          timestamp: new Date()
        }
      ]);
    }
  }, [isOpen, messages.length]);

  // Scroll to bottom of messages
  useEffect(() => {
    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages]);

  // Handle text selection - only run in browser environment
  useEffect(() => {
    // Safety check for server-side rendering
    if (typeof window === 'undefined' || !window.document) {
      return;
    }

    try {
      const handleSelection = () => {
        try {
          const selection = window.getSelection();
          if (!selection) return;

          const selectedText = selection.toString().trim();

          if (selectedText.length > 0 && selection.anchorOffset !== selection.focusOffset) {
            const range = selection.getRangeAt(0);
            const rect = range.getBoundingClientRect();

            if (rect && rect.width > 0 && rect.height > 0) {
              setTextSelectionPosition({
                x: rect.left + window.scrollX,
                y: rect.top + window.scrollY - 40 // Position above the selected text
              });
              setSelectedText(selectedText);
              setShowTextSelectionButton(true);

              // Clear any existing timeout
              if (textSelectionTimeout.current) {
                clearTimeout(textSelectionTimeout.current);
              }

              // Set timeout to hide the button after a delay
              textSelectionTimeout.current = setTimeout(() => {
                setShowTextSelectionButton(false);
              }, 3000);
            }
          } else {
            setShowTextSelectionButton(false);
          }
        } catch (e) {
          // Ignore errors that might occur during selection
          console.warn('Text selection handling error:', e);
          setShowTextSelectionButton(false);
        }
      };

      const handleEscape = (e: KeyboardEvent) => {
        if (e.key === 'Escape') {
          setShowTextSelectionButton(false);
        }
      };

      document.addEventListener('mouseup', handleSelection);
      document.addEventListener('keyup', handleEscape);

      return () => {
        document.removeEventListener('mouseup', handleSelection);
        document.removeEventListener('keyup', handleEscape);

        if (textSelectionTimeout.current) {
          clearTimeout(textSelectionTimeout.current);
        }
      };
    } catch (e) {
      console.error('Error initializing text selection handler:', e);
    }
  }, []);

  const handleQuerySubmit = async (queryText: string): Promise<void> => {
    if (!queryText.trim()) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      text: queryText,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setLoading(true);
    setError(null);
    setNotification(null);

    try {
      // For now, using a simple fallback since the real API might cause issues
      // In a real implementation, you would connect to the actual backend API
      console.log('Query submitted:', queryText);

      // Simulate response after a delay
      await new Promise(resolve => setTimeout(resolve, 1000));

      // Add AI response
      const aiResponse: Message = {
        id: (Date.now() + 1).toString(),
        text: `This is a simulated response to: "${queryText}". In the full implementation, this would connect to the backend API and provide a real answer based on the Physical AI and Humanoid Robotics book content.`,
        sender: 'ai',
        timestamp: new Date()
      };

      setMessages(prev => [...prev, aiResponse]);

      setNotification({
        message: 'Query processed successfully!',
        type: 'success'
      });
    } catch (err: unknown) {
      const errorMessage = typeof err === 'string' ? err : 'Failed to process query';
      setError(errorMessage);
      setNotification({
        message: errorMessage,
        type: 'error'
      });
      console.error('Query submission error:', err);
    } finally {
      setLoading(false);
    }
  };

  const handleAskFromAI = () => {
    setIsOpen(true);
    setShowTextSelectionButton(false);
    // Submit the selected text as a query
    if (selectedText) {
      handleQuerySubmit(selectedText);
    }
  };

  const clearNotification = (): void => {
    setNotification(null);
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const closeChat = () => {
    setIsOpen(false);
  };

  // Handle Enter key for message submission
  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleQuerySubmit(inputValue);
    }
  };

  // Try to dynamically import components, with fallbacks
  const [QueryInputComponent, setQueryInputComponent] = useState<any>(null);
  const [ErrorNotifierComponent, setErrorNotifierComponent] = useState<any>(null);

  useEffect(() => {
    const loadComponents = async () => {
      try {
        // Attempt to load the real components
        const { default: QueryInput } = await import('./QueryInput');
        const { default: ErrorNotifier } = await import('./ErrorNotifier');

        setQueryInputComponent(() => QueryInput);
        setErrorNotifierComponent(() => ErrorNotifier);
      } catch (err) {
        console.warn('Failed to load components, using fallbacks:', err);
        // Use fallback components if imports fail
        setQueryInputComponent(() => FallbackQueryInput);
        setErrorNotifierComponent(() => FallbackErrorNotifier);
      }
    };

    loadComponents();
  }, []);

  if (!QueryInputComponent || !ErrorNotifierComponent) {
    // Render nothing or minimal UI while components are loading
    return (
      <>
        {/* Minimal floating button that works without other components */}
        <button
          className={`floating-chat-button ${isOpen ? 'hidden' : 'visible'}`}
          onClick={toggleChat}
          aria-label="Open AI Assistant"
          title="Open AI Assistant"
          style={{
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            width: '60px',
            height: '60px',
            borderRadius: '50%',
            backgroundColor: '#3578e5',
            color: 'white',
            border: 'none',
            cursor: 'pointer',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
            zIndex: 9999,
            fontSize: '24px'
          }}
        >
          <svg
            xmlns="http://www.w3.org/2000/svg"
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
          </svg>
        </button>
      </>
    );
  }

  return (
    <>
      {/* Floating chat button that appears on every page */}
      <button
        className={`floating-chat-button ${isOpen ? 'hidden' : 'visible'}`}
        onClick={toggleChat}
        aria-label="Open AI Assistant"
        title="Open AI Assistant"
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          width="24"
          height="24"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
          className="chat-icon"
        >
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
        </svg>
      </button>

      {/* Text selection "Ask from AI" button */}
      {showTextSelectionButton && (
        <button
          className="text-selection-ask-button"
          style={{
            position: 'fixed',
            left: `${textSelectionPosition.x}px`,
            top: `${textSelectionPosition.y}px`,
            zIndex: 10000,
          }}
          onClick={handleAskFromAI}
          aria-label="Ask AI about selected text"
        >
          Ask from AI
        </button>
      )}

      {/* Floating chat window */}
      {isOpen && (
        <div className="floating-chat-window">
          <div className="chat-header">
            <h3>AI Assistant</h3>
            <button
              className="close-button"
              onClick={closeChat}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>

          {notification && (
            <ErrorNotifierComponent
              message={notification.message}
              type={notification.type}
              visible={true}
              onClose={clearNotification}
            />
          )}

          <div className="chat-messages">
            {messages.map((message) => (
              <div
                key={message.id}
                className={`message-bubble ${message.sender}-message`}
              >
                <div className="message-text">{message.text}</div>
                <div className="message-time">
                  {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                </div>
              </div>
            ))}
            {loading && (
              <div className="message-bubble ai-message">
                <div className="message-text">
                  <span className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chat-input-section">
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Type a message..."
              disabled={loading}
              className="chat-input"
            />
            <button
              onClick={() => handleQuerySubmit(inputValue)}
              disabled={loading || !inputValue.trim()}
              className="send-button"
            >
              <svg
                xmlns="http://www.w3.org/2000/svg"
                width="20"
                height="20"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              >
                <line x1="22" y1="2" x2="11" y2="13"></line>
                <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
              </svg>
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default FloatingChatbot;