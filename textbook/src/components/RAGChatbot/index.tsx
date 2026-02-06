import React, { useState, useEffect, useCallback, useRef } from "react";
import styles from "./styles.module.css";

// API configuration - Update this when you deploy the chatbot API
const API_BASE_URL =
  typeof window !== "undefined" && window.location.hostname !== "localhost"
    ? "https://physical-ai-chatbot.vercel.app/api/v1"
    : "http://localhost:8000/api/v1";

interface Message {
  id: string;
  role: "user" | "assistant";
  content: string;
  sources?: Array<{
    chapter: string;
    section: string;
    relevance: number;
  }>;
  timestamp: Date;
}

function ChatIcon() {
  return (
    <svg viewBox="0 0 24 24" fill="currentColor" width="28" height="28">
      <path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm0 14H6l-2 2V4h16v12z" />
      <circle cx="12" cy="10" r="1.5" />
      <circle cx="8" cy="10" r="1.5" />
      <circle cx="16" cy="10" r="1.5" />
    </svg>
  );
}

function CloseIcon() {
  return (
    <svg viewBox="0 0 24 24" fill="currentColor" width="20" height="20">
      <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z" />
    </svg>
  );
}

function SendIcon() {
  return (
    <svg viewBox="0 0 24 24" fill="currentColor" width="20" height="20">
      <path d="M2.01 21L23 12 2.01 3 2 10l15 2-15 2z" />
    </svg>
  );
}

export default function RAGChatbot() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    {
      id: "welcome",
      role: "assistant",
      content:
        "Hello! I'm your Physical AI & Robotics assistant. Ask me anything about the textbook content, ROS 2, simulation, or humanoid robotics. You can also select text on the page and ask me about it!",
      timestamp: new Date(),
    },
  ]);
  const [input, setInput] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [sessionId, setSessionId] = useState<string>(
    () => `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
  );
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages]);

  // Listen for text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 10 && text.length < 1000) {
        setSelectedText(text);
      }
    };

    document.addEventListener("mouseup", handleSelection);
    return () => document.removeEventListener("mouseup", handleSelection);
  }, []);

  const sendMessage = async (content: string) => {
    if (isLoading || !content.trim()) return;

    const userMessage: Message = {
      id: `user_${Date.now()}`,
      role: "user",
      content,
      timestamp: new Date(),
    };
    setMessages((prev) => [...prev, userMessage]);
    setInput("");
    setIsLoading(true);

    try {
      const response = await fetch(`${API_BASE_URL}/chat/`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          message: content,
          session_id: sessionId,
          selected_text: selectedText,
        }),
      });

      if (!response.ok) {
        throw new Error("Failed to get response");
      }

      const data = await response.json();

      const assistantMessage: Message = {
        id: `assistant_${Date.now()}`,
        role: "assistant",
        content: data.message,
        sources: data.sources,
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, assistantMessage]);

      // Clear selected text after using it
      if (selectedText) {
        setSelectedText(null);
      }
    } catch (error) {
      console.error("Chat error:", error);

      // Fallback response when API is not available
      const fallbackMessage: Message = {
        id: `assistant_${Date.now()}`,
        role: "assistant",
        content:
          "I apologize, but I'm currently unable to connect to the knowledge base. The chatbot API may not be deployed yet. Please try again later or contact the administrator.",
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, fallbackMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleAskAboutSelection = async () => {
    if (!selectedText) return;

    setIsLoading(true);
    const userMessage: Message = {
      id: `user_${Date.now()}`,
      role: "user",
      content: `Please explain this selected text: "${selectedText.substring(0, 100)}${selectedText.length > 100 ? "..." : ""}"`,
      timestamp: new Date(),
    };
    setMessages((prev) => [...prev, userMessage]);

    try {
      const response = await fetch(`${API_BASE_URL}/chat/selected-text`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          selected_text: selectedText,
        }),
      });

      if (!response.ok) {
        throw new Error("Failed to get response");
      }

      const data = await response.json();

      const assistantMessage: Message = {
        id: `assistant_${Date.now()}`,
        role: "assistant",
        content: data.message,
        sources: data.sources,
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, assistantMessage]);
      setSelectedText(null);
    } catch (error) {
      console.error("Selection query error:", error);

      const fallbackMessage: Message = {
        id: `assistant_${Date.now()}`,
        role: "assistant",
        content:
          "I apologize, but I'm currently unable to analyze the selected text. The chatbot API may not be deployed yet.",
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, fallbackMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    sendMessage(input);
  };

  const handleClearSelection = () => {
    setSelectedText(null);
    window.getSelection()?.removeAllRanges();
  };

  return (
    <div className={styles.chatbotContainer}>
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <span className={styles.chatTitle}>📚 AI Assistant</span>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
            >
              <CloseIcon />
            </button>
          </div>

          {selectedText && (
            <div className={styles.selectedTextBanner}>
              <span className={styles.selectedTextLabel}>Selected text:</span>
              <span className={styles.selectedTextPreview}>
                "{selectedText.substring(0, 50)}
                {selectedText.length > 50 ? "..." : ""}"
              </span>
              <div className={styles.selectedTextActions}>
                <button onClick={handleAskAboutSelection} disabled={isLoading}>
                  Explain this
                </button>
                <button onClick={handleClearSelection}>Clear</button>
              </div>
            </div>
          )}

          <div className={styles.messagesContainer}>
            {messages.map((message) => (
              <div
                key={message.id}
                className={`${styles.message} ${styles[message.role]}`}
              >
                <div className={styles.messageContent}>{message.content}</div>
                {message.sources && message.sources.length > 0 && (
                  <div className={styles.sources}>
                    <span className={styles.sourcesLabel}>Sources:</span>
                    {message.sources.map((source, idx) => (
                      <span key={idx} className={styles.sourceChip}>
                        {source.chapter} ({source.relevance}%)
                      </span>
                    ))}
                  </div>
                )}
              </div>
            ))}
            {isLoading && (
              <div className={`${styles.message} ${styles.assistant}`}>
                <div className={styles.typingIndicator}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <form className={styles.inputForm} onSubmit={handleSubmit}>
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              placeholder="Ask about the textbook..."
              disabled={isLoading}
            />
            <button type="submit" disabled={isLoading || !input.trim()}>
              <SendIcon />
            </button>
          </form>
        </div>
      )}

      <button
        className={`${styles.chatButton} ${isOpen ? styles.open : ""}`}
        onClick={() => setIsOpen(!isOpen)}
        title={isOpen ? "Close chat" : "Open AI assistant"}
      >
        {isOpen ? <CloseIcon /> : <ChatIcon />}
      </button>
    </div>
  );
}
