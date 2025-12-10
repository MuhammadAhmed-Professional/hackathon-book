import React, { createContext, useContext, useState, ReactNode } from 'react';

interface ChatbotContextType {
  isOpen: boolean;
  selectedText: string | undefined;
  openChat: (text?: string) => void;
  closeChat: () => void;
  clearSelectedText: () => void;
}

const ChatbotContext = createContext<ChatbotContextType | undefined>(undefined);

export const useChatbot = () => {
  const context = useContext(ChatbotContext);
  if (!context) {
    throw new Error('useChatbot must be used within ChatbotProvider');
  }
  return context;
};

interface ChatbotProviderProps {
  children: ReactNode;
}

export const ChatbotProvider: React.FC<ChatbotProviderProps> = ({ children }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [selectedText, setSelectedText] = useState<string | undefined>(undefined);

  const openChat = (text?: string) => {
    if (text) {
      setSelectedText(text);
    }
    setIsOpen(true);
  };

  const closeChat = () => {
    setIsOpen(false);
    // Don't clear selectedText immediately to allow for smooth transitions
    setTimeout(() => {
      setSelectedText(undefined);
    }, 300);
  };

  const clearSelectedText = () => {
    setSelectedText(undefined);
  };

  return (
    <ChatbotContext.Provider
      value={{
        isOpen,
        selectedText,
        openChat,
        closeChat,
        clearSelectedText,
      }}
    >
      {children}
    </ChatbotContext.Provider>
  );
};
