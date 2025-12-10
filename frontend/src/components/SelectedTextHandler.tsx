import React, { useState, useEffect } from 'react';
import Chatbot from './Chatbot';

const SelectedTextHandler: React.FC = () => {
  const [selectedText, setSelectedText] = useState<string>('');
  const [showButton, setShowButton] = useState(false);
  const [buttonPosition, setButtonPosition] = useState({ top: 0, left: 0 });

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();

      if (selection && selection.toString().trim().length > 0) {
        const text = selection.toString().trim();

        // Validate minimum length
        if (text.length >= 10) {
          setSelectedText(text);

          // Get selection range position
          const range = selection.getRangeAt(0);
          const rect = range.getBoundingClientRect();

          // Position button near selection
          setButtonPosition({
            top: rect.bottom + window.scrollY + 5,
            left: rect.left + window.scrollX
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
    // The Chatbot component will receive selectedText as prop
    setShowButton(false);
    // Scroll to chatbot if it exists
    const chatbotElement = document.querySelector('.chatbot-container');
    if (chatbotElement) {
      chatbotElement.scrollIntoView({ behavior: 'smooth', block: 'start' });
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
          className="selected-text-button-container"
          style={{
            position: 'absolute',
            top: `${buttonPosition.top}px`,
            left: `${buttonPosition.left}px`,
            zIndex: 1000,
          }}
        >
          <button
            className="selected-text-button"
            onClick={handleAskAboutSelection}
            onMouseDown={(e) => e.preventDefault()} // Prevent text deselection
          >
            Ask about this selection
          </button>
          <button
            className="selected-text-close"
            onClick={handleClearSelection}
            onMouseDown={(e) => e.preventDefault()}
          >
            ×
          </button>
        </div>
      )}
      <Chatbot selectedText={selectedText} />
    </>
  );
};

export default SelectedTextHandler;

