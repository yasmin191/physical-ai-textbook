import React, { useState, useEffect, useCallback } from 'react';
import ChatWindow from './ChatWindow';
import { Message, Source } from './MessageList';
import styles from './styles.module.css';

// API configuration
const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-api-domain.vercel.app/api/v1'
  : 'http://localhost:8000/api/v1';

function ChatIcon() {
  return (
    <svg viewBox="0 0 24 24" fill="currentColor">
      <path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm0 14H6l-2 2V4h16v12z" />
      <circle cx="12" cy="10" r="1.5" />
      <circle cx="8" cy="10" r="1.5" />
      <circle cx="16" cy="10" r="1.5" />
    </svg>
  );
}

interface ChatResponse {
  message_id: string;
  content: string;
  language: string;
  sources: Source[];
  tokens_used: number;
}

export default function RAGChatbot() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [language, setLanguage] = useState('en');
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [sessionId, setSessionId] = useState<string | null>(null);

  // Create session on mount
  useEffect(() => {
    const createSession = async () => {
      try {
        const response = await fetch(`${API_BASE_URL}/chat/sessions`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
        });
        if (response.ok) {
          const data = await response.json();
          setSessionId(data.id);
        }
      } catch (error) {
        console.error('Failed to create chat session:', error);
      }
    };
    createSession();
  }, []);

  // Listen for text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 10 && text.length < 500) {
        setSelectedText(text);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  // Get current chapter context from URL
  const getChapterContext = useCallback((): string | null => {
    const path = window.location.pathname;
    const match = path.match(/\/docs\/([^/]+\/[^/]+)/);
    return match ? match[1] : null;
  }, []);

  const sendMessage = async (content: string) => {
    if (isLoading) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content,
      timestamp: new Date(),
    };
    setMessages((prev) => [...prev, userMessage]);
    setIsLoading(true);

    try {
      const response = await fetch(`${API_BASE_URL}/chat/`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          message: content,
          session_id: sessionId,
          language,
          selected_text: selectedText,
          chapter_context: getChapterContext(),
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to get response');
      }

      const data: ChatResponse = await response.json();

      // Add assistant message
      const assistantMessage: Message = {
        id: data.message_id,
        role: 'assistant',
        content: data.content,
        sources: data.sources,
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, assistantMessage]);

      // Clear selected text after using it
      if (selectedText) {
        setSelectedText(null);
      }
    } catch (error) {
      console.error('Chat error:', error);

      // Add error message
      const errorMessage: Message = {
        id: Date.now().toString() + '-error',
        role: 'assistant',
        content: language === 'ur'
          ? 'معذرت، ایک خرابی پیش آگئی۔ براہ کرم دوبارہ کوشش کریں۔'
          : 'Sorry, something went wrong. Please try again.',
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleClearSelection = () => {
    setSelectedText(null);
    window.getSelection()?.removeAllRanges();
  };

  return (
    <div className={styles.chatbotContainer}>
      {isOpen ? (
        <ChatWindow
          messages={messages}
          isLoading={isLoading}
          language={language}
          selectedText={selectedText}
          onSend={sendMessage}
          onClose={() => setIsOpen(false)}
          onClearSelection={handleClearSelection}
          onLanguageChange={setLanguage}
        />
      ) : null}
      <button
        className={styles.chatButton}
        onClick={() => setIsOpen(!isOpen)}
        title={isOpen ? 'Close chat' : 'Open chat assistant'}
      >
        <ChatIcon />
      </button>
    </div>
  );
}
