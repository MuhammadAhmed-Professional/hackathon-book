import React, { useState, useEffect } from 'react';
import Chatbot from './Chatbot';
import { useChatbot } from './ChatbotContext';
import './FloatingChatbot.css';

const FloatingChatbot: React.FC = () => {
  const { isOpen, selectedText, openChat, closeChat, clearSelectedText } = useChatbot();
  const [isMinimized, setIsMinimized] = useState(false);

  // Reset minimized state when chat is closed
  useEffect(() => {
    if (!isOpen) {
      setIsMinimized(false);
    }
  }, [isOpen]);

  const toggleChat = () => {
    if (isOpen && !isMinimized) {
      setIsMinimized(true);
    } else if (isOpen && isMinimized) {
      setIsMinimized(false);
    } else {
      openChat();
      setIsMinimized(false);
    }
  };

  const handleClose = () => {
    closeChat();
    setIsMinimized(false);
  };

  return (
    <>
      {/* Floating Button */}
      <button
        className={`floating-chat-button ${isOpen ? 'active' : ''}`}
        onClick={toggleChat}
        aria-label="Open AI Assistant"
        title="AI Learning Assistant"
      >
        {isOpen && !isMinimized ? (
          // Minimize icon
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round">
            <line x1="5" y1="12" x2="19" y2="12"></line>
          </svg>
        ) : (
          // Chat icon
          <svg width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
            <path d="M13 8H7"></path>
            <path d="M17 12H7"></path>
            <path d="M17 16H7"></path>
          </svg>
        )}
        {!isOpen && (
          <span className="floating-chat-badge">AI</span>
        )}
      </button>

      {/* Chat Modal */}
      {isOpen && (
        <div className={`floating-chat-modal ${isMinimized ? 'minimized' : ''}`}>
          {/* Header */}
          <div className="chat-modal-header">
            <div className="chat-modal-title">
              <div className="chat-modal-avatar">
                <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                  <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
                  <path d="M13 8H7"></path>
                  <path d="M17 12H7"></path>
                  <path d="M17 16H7"></path>
                </svg>
              </div>
              <div>
                <h3>AI Learning Assistant</h3>
                <span className="chat-modal-subtitle">
                  {selectedText ? 'Context Mode: Selected Text' : 'Ask me anything about robotics!'}
                </span>
              </div>
            </div>
            <div className="chat-modal-actions">
              <button
                className="chat-modal-action-btn"
                onClick={() => setIsMinimized(!isMinimized)}
                aria-label={isMinimized ? 'Maximize' : 'Minimize'}
                title={isMinimized ? 'Maximize' : 'Minimize'}
              >
                {isMinimized ? (
                  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round">
                    <polyline points="15 3 21 3 21 9"></polyline>
                    <polyline points="9 21 3 21 3 15"></polyline>
                    <line x1="21" y1="3" x2="14" y2="10"></line>
                    <line x1="3" y1="21" x2="10" y2="14"></line>
                  </svg>
                ) : (
                  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round">
                    <line x1="5" y1="12" x2="19" y2="12"></line>
                  </svg>
                )}
              </button>
              <button
                className="chat-modal-action-btn close"
                onClick={handleClose}
                aria-label="Close"
                title="Close"
              >
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round">
                  <line x1="18" y1="6" x2="6" y2="18"></line>
                  <line x1="6" y1="6" x2="18" y2="18"></line>
                </svg>
              </button>
            </div>
          </div>

          {/* Chat Content */}
          {!isMinimized && (
            <div className="chat-modal-content">
              <Chatbot selectedText={selectedText} />
            </div>
          )}

          {/* Feature Pills (when maximized) */}
          {!isMinimized && (
            <div className="chat-modal-features">
              <span className="feature-pill">
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round">
                  <path d="M13 2L3 14h9l-1 8 10-12h-9l1-8z"></path>
                </svg>
                RAG-Powered
              </span>
              <span className="feature-pill">
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round">
                  <circle cx="12" cy="12" r="10"></circle>
                  <polyline points="12 6 12 12 16 14"></polyline>
                </svg>
                Real-time
              </span>
              <span className="feature-pill">
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round">
                  <path d="M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z"></path>
                </svg>
                Secure
              </span>
            </div>
          )}
        </div>
      )}

      {/* Overlay (for mobile) */}
      {isOpen && !isMinimized && (
        <div className="chat-modal-overlay" onClick={handleClose}></div>
      )}
    </>
  );
};

export default FloatingChatbot;
