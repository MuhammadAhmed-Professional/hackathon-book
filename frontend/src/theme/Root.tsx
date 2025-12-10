/**
 * Root wrapper for the entire Docusaurus app.
 * Provides global context and components:
 * - ChatbotProvider: Global chatbot state management
 * - FloatingChatbot: Floating AI assistant button
 */

import React from 'react';
import { ChatbotProvider } from '../components/ChatbotContext';
import FloatingChatbot from '../components/FloatingChatbot';

export default function Root({ children }) {
  return (
    <ChatbotProvider>
      {children}
      <FloatingChatbot />
    </ChatbotProvider>
  );
}
