import React, { useState, useRef, useEffect } from 'react';
import config from '../config';
import './Chatbot.css';

interface Message {
  id: string;
  text: string;
  isUser: boolean;
  sources?: Source[];
  timestamp: Date;
  conversationId?: number;
  skillOutput?: boolean;
  skillName?: string;
}

interface Source {
  chunk_id: string;
  chapter_id: string;
  relevance_score: number;
  snippet: string;
}

// Generate or retrieve session ID for anonymous users
const getSessionId = (): string => {
  let sessionId = localStorage.getItem('session_id');
  if (!sessionId) {
    sessionId = crypto.randomUUID();
    localStorage.setItem('session_id', sessionId);
  }
  return sessionId;
};

// Check if user is authenticated (JWT token exists)
const getAuthToken = (): string | null => {
  return localStorage.getItem('auth_token');
};

const Chatbot: React.FC<{ selectedText?: string }> = ({ selectedText: initialSelectedText }) => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const [selectedText, setSelectedText] = useState<string | undefined>(initialSelectedText);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  
  // Update selectedText when prop changes
  useEffect(() => {
    if (initialSelectedText) {
      setSelectedText(initialSelectedText);
    }
  }, [initialSelectedText]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessage = async () => {
    if (!input.trim() || loading) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      text: input,
      isUser: true,
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    const questionText = input;
    setInput('');
    setLoading(true);

    try {
      const endpoint = selectedText ? '/ask_selected' : '/ask';

      // Get session ID for anonymous users or auth token for authenticated users
      const authToken = getAuthToken();
      const sessionId = authToken ? undefined : getSessionId();

      const body = selectedText
        ? { question: questionText, context: selectedText, session_id: sessionId }
        : { question: questionText, session_id: sessionId };

      // Prepare headers
      const headers: HeadersInit = {
        'Content-Type': 'application/json',
      };

      // Add Authorization header if user is authenticated
      if (authToken) {
        headers['Authorization'] = `Bearer ${authToken}`;
      }

      const response = await fetch(`${config.API_BASE_URL}${endpoint}`, {
        method: 'POST',
        headers,
        body: JSON.stringify(body),
      });

      if (!response.ok) {
        throw new Error('Failed to get answer');
      }

      const data = await response.json();

      const botMessage: Message = {
        id: (Date.now() + 1).toString(),
        text: data.answer_text,
        isUser: false,
        sources: data.sources || [],
        timestamp: new Date(),
        conversationId: data.conversation_id,
      };

      // Log conversation ID for debugging
      if (data.conversation_id) {
        console.log(`Conversation saved: ID=${data.conversation_id}`);
      }

      setMessages((prev) => [...prev, botMessage]);
    } catch (error) {
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        text: 'Sorry, I encountered an error. Please try again later.',
        isUser: false,
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  // Agent Skills Integration
  const executeSkill = async (skillType: 'summarize' | 'quiz' | 'explain', data: any) => {
    setLoading(true);
    try {
      const authToken = getAuthToken();
      const headers: HeadersInit = {
        'Content-Type': 'application/json',
      };
      if (authToken) {
        headers['Authorization'] = `Bearer ${authToken}`;
      }

      const response = await fetch(`${config.API_BASE_URL}/skills/${skillType}`, {
        method: 'POST',
        headers,
        body: JSON.stringify(data),
      });

      if (!response.ok) {
        throw new Error(`Skill execution failed: ${skillType}`);
      }

      const result = await response.json();

      const skillMessage: Message = {
        id: Date.now().toString(),
        text: result.output,
        isUser: false,
        timestamp: new Date(),
        skillOutput: true,
        skillName: result.skill_name,
      };

      setMessages((prev) => [...prev, skillMessage]);
    } catch (error) {
      const errorMessage: Message = {
        id: Date.now().toString(),
        text: `Sorry, I couldn't execute the ${skillType} skill. Please try again.`,
        isUser: false,
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setLoading(false);
    }
  };

  const handleExplainTerm = () => {
    const term = prompt('Enter the term to explain:');
    if (term && term.trim()) {
      const userMessage: Message = {
        id: Date.now().toString(),
        text: `Explain: "${term}"`,
        isUser: true,
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, userMessage]);

      executeSkill('explain', {
        term: term.trim(),
        context: selectedText || null,
        tier: null,
      });
    }
  };

  const handleSummarize = () => {
    const content = selectedText || prompt('Paste the section content to summarize:');
    if (content && content.trim()) {
      const userMessage: Message = {
        id: Date.now().toString(),
        text: `Summarize this section`,
        isUser: true,
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, userMessage]);

      executeSkill('summarize', {
        content: content.trim(),
        tier: null,
      });
    }
  };

  const handleGenerateQuiz = () => {
    const content = selectedText || prompt('Paste the chapter content for quiz generation:');
    if (content && content.trim()) {
      const userMessage: Message = {
        id: Date.now().toString(),
        text: `Generate quiz questions`,
        isUser: true,
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, userMessage]);

      executeSkill('quiz', {
        content: content.trim(),
        num_questions: 5,
        difficulty: null,
      });
    }
  };

  return (
    <div className="chatbot-container">
      {selectedText && (
        <div className="chatbot-context-notice">
          <strong>Context Mode:</strong> Answering based on selected text only
        </div>
      )}
      <div className="chatbot-messages">
        {messages.length === 0 && (
            <div className="chatbot-welcome">
            <p>
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" style={{ display: 'inline-block', verticalAlign: 'middle', marginRight: '8px' }}>
                <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
              </svg>
              Ask me anything about the book!
            </p>
            <p className="chatbot-hint">
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" style={{ display: 'inline-block', verticalAlign: 'middle', marginRight: '6px' }}>
                <circle cx="12" cy="12" r="10"></circle>
                <line x1="12" y1="16" x2="12" y2="12"></line>
                <line x1="12" y1="8" x2="12.01" y2="8"></line>
              </svg>
              Try: "What is ROS 2?" or "Explain humanoid robotics"
            </p>
            <div className="skill-quick-actions">
              <p className="skill-actions-title">AI Skills:</p>
              <div className="skill-buttons">
                <button className="skill-button explain" onClick={handleExplainTerm}>
                  <svg className="skill-icon" width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                    <circle cx="11" cy="11" r="8"></circle>
                    <path d="m21 21-4.35-4.35"></path>
                  </svg>
                  Explain Term
                </button>
                <button className="skill-button summarize" onClick={handleSummarize}>
                  <svg className="skill-icon" width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                    <path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z"></path>
                    <path d="M14 2v6h6"></path>
                    <path d="M16 13H8"></path>
                    <path d="M16 17H8"></path>
                    <path d="M10 9H8"></path>
                  </svg>
                  Summarize
                </button>
                <button className="skill-button quiz" onClick={handleGenerateQuiz}>
                  <svg className="skill-icon" width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                    <line x1="18" y1="20" x2="18" y2="10"></line>
                    <line x1="12" y1="20" x2="12" y2="4"></line>
                    <line x1="6" y1="20" x2="6" y2="14"></line>
                  </svg>
                  Generate Quiz
                </button>
              </div>
            </div>
          </div>
        )}
        {messages.map((message) => (
          <div
            key={message.id}
            className={`chatbot-message ${message.isUser ? 'user' : 'bot'} ${message.skillOutput ? 'skill-output' : ''}`}
          >
            {!message.isUser && (
              <div className="message-avatar bot-avatar">
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                  <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
                  <path d="M13 8H7"></path>
                  <path d="M17 12H7"></path>
                  <path d="M17 16H7"></path>
                </svg>
              </div>
            )}
            <div className="message-content">
              {message.skillOutput && (
                <div className="skill-badge">
                  <svg className="skill-badge-icon" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                    <polygon points="12 2 15.09 8.26 22 9.27 17 14.14 18.18 21.02 12 17.77 5.82 21.02 7 14.14 2 9.27 8.91 8.26 12 2"></polygon>
                  </svg>
                  <span>{message.skillName || 'AI Skill'}</span>
                </div>
              )}
              <p className={message.skillOutput ? 'skill-text' : ''}>{message.text}</p>
              {message.sources && message.sources.length > 0 && (
                <div className="message-sources">
                  <strong>Sources:</strong>
                  <ul>
                    {message.sources.map((source, idx) => (
                      <li key={idx}>
                        <a
                          href={`/docs/${source.chapter_id}`}
                          target="_blank"
                          rel="noopener noreferrer"
                        >
                          {source.chapter_id}
                        </a>
                        {' '}
                        <span className="relevance-score">({(source.relevance_score * 100).toFixed(1)}% match)</span>
                      </li>
                    ))}
                  </ul>
                </div>
              )}
            </div>
            {message.isUser && (
              <div className="message-avatar user-avatar">
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                  <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2"></path>
                  <circle cx="12" cy="7" r="4"></circle>
                </svg>
              </div>
            )}
          </div>
        ))}
        {loading && (
          <div className="chatbot-message bot">
            <div className="message-content">
              <div className="loading-dots">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>
      <div className="chatbot-input-container">
        <textarea
          className="chatbot-input"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Ask a question about the book..."
          rows={2}
        />
        <button
          className="chatbot-send-button"
          onClick={sendMessage}
          disabled={loading || !input.trim()}
          title={loading ? 'Sending...' : 'Send message'}
        >
          {loading ? (
            <>
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round" style={{ marginRight: '8px' }}>
                <line x1="12" y1="2" x2="12" y2="6"></line>
                <line x1="12" y1="18" x2="12" y2="22"></line>
                <line x1="4.93" y1="4.93" x2="7.76" y2="7.76"></line>
                <line x1="16.24" y1="16.24" x2="19.07" y2="19.07"></line>
                <line x1="2" y1="12" x2="6" y2="12"></line>
                <line x1="18" y1="12" x2="22" y2="12"></line>
                <line x1="4.93" y1="19.07" x2="7.76" y2="16.24"></line>
                <line x1="16.24" y1="7.76" x2="19.07" y2="4.93"></line>
              </svg>
              Sending...
            </>
          ) : (
            <>
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round" style={{ marginRight: '8px' }}>
                <line x1="22" y1="2" x2="11" y2="13"></line>
                <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
              </svg>
              Send
            </>
          )}
        </button>
      </div>
    </div>
  );
};

export default Chatbot;

