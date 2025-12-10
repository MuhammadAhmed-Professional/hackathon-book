import React, { useState, useEffect } from 'react';
import { useChatbot } from './ChatbotContext';
import './TextSelectionHandler.css';

/**
 * TextSelectionHandler - Detects text selection and shows a button
 * to open the floating chatbot with the selected text.
 */
const TextSelectionHandler: React.FC = () => {
  const { openChat } = useChatbot();
  const [showButton, setShowButton] = useState(false);
  const [buttonPosition, setButtonPosition] = useState({ top: 0, left: 0 });
  const [selectedText, setSelectedText] = useState<string>('');

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();

      if (selection && selection.toString().trim().length > 0) {
        const text = selection.toString().trim();

        // Validate minimum length (at least 3 characters)
        if (text.length >= 3) {
          setSelectedText(text);

          // Get selection range position
          const range = selection.getRangeAt(0);
          const rect = range.getBoundingClientRect();

          // Position button near selection (above or below)
          setButtonPosition({
            top: rect.bottom + window.scrollY + 5,
            left: rect.left + window.scrollX + (rect.width / 2) - 100, // Center button over selection
          });

          setShowButton(true);
        } else {
          setShowButton(false);
        }
      } else {
        setShowButton(false);
        setSelectedText('');
      }
    };

    // Listen for text selection
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, []);

  const handleAskAboutSelection = () => {
    if (selectedText) {
      // Open the floating chatbot with the selected text
      openChat(selectedText);
      setShowButton(false);
      // Clear the selection
      window.getSelection()?.removeAllRanges();
    }
  };

  const handleClearSelection = () => {
    setSelectedText('');
    setShowButton(false);
    window.getSelection()?.removeAllRanges();
  };

  return (
    <>
      {showButton && selectedText && (
        <div
          className="text-selection-button-container"
          style={{
            position: 'absolute',
            top: `${buttonPosition.top}px`,
            left: `${buttonPosition.left}px`,
            zIndex: 10000,
          }}
        >
          <button
            className="text-selection-button"
            onClick={handleAskAboutSelection}
            onMouseDown={(e) => e.preventDefault()} // Prevent text deselection
            title="Ask AI about this selection"
          >
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
            </svg>
            Ask AI about this
          </button>
          <button
            className="text-selection-close"
            onClick={handleClearSelection}
            onMouseDown={(e) => e.preventDefault()}
            title="Clear selection"
          >
            ×
          </button>
        </div>
      )}
    </>
  );
};

export default TextSelectionHandler;

