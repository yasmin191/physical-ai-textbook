import React, { useEffect, useRef } from 'react';
import styles from './styles.module.css';

export interface Source {
  chapter_slug: string;
  section_title: string;
  chapter_title: string;
  relevance_score: number;
  text_snippet?: string;
}

export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Source[];
  timestamp: Date;
}

interface MessageListProps {
  messages: Message[];
  isLoading: boolean;
  language: string;
}

function TypingIndicator() {
  return (
    <div className={`${styles.message} ${styles.assistantMessage}`}>
      <div className={styles.typingIndicator}>
        <span className={styles.typingDot}></span>
        <span className={styles.typingDot}></span>
        <span className={styles.typingDot}></span>
      </div>
    </div>
  );
}

function SourcesList({ sources }: { sources: Source[] }) {
  if (!sources || sources.length === 0) return null;

  return (
    <div className={styles.sources}>
      <div className={styles.sourcesTitle}>Sources:</div>
      {sources.map((source, index) => (
        <a
          key={index}
          href={`/docs/${source.chapter_slug}`}
          className={styles.sourceLink}
          target="_blank"
          rel="noopener noreferrer"
        >
          📖 {source.chapter_title} - {source.section_title}
        </a>
      ))}
    </div>
  );
}

function MessageItem({ message, language }: { message: Message; language: string }) {
  const isUser = message.role === 'user';
  const isRTL = language === 'ur';

  return (
    <div
      className={`${styles.message} ${isUser ? styles.userMessage : styles.assistantMessage}`}
      dir={isRTL ? 'rtl' : 'ltr'}
    >
      <div className={styles.messageContent}>{message.content}</div>
      {!isUser && message.sources && <SourcesList sources={message.sources} />}
    </div>
  );
}

function EmptyState() {
  return (
    <div className={styles.emptyState}>
      <svg
        className={styles.emptyStateIcon}
        viewBox="0 0 24 24"
        fill="none"
        stroke="currentColor"
        strokeWidth="1.5"
      >
        <path
          strokeLinecap="round"
          strokeLinejoin="round"
          d="M12 6.042A8.967 8.967 0 006 3.75c-1.052 0-2.062.18-3 .512v14.25A8.987 8.987 0 016 18c2.305 0 4.408.867 6 2.292m0-14.25a8.966 8.966 0 016-2.292c1.052 0 2.062.18 3 .512v14.25A8.987 8.987 0 0018 18a8.967 8.967 0 00-6 2.292m0-14.25v14.25"
        />
      </svg>
      <div className={styles.emptyStateTitle}>Ask about the textbook</div>
      <div className={styles.emptyStateText}>
        I can help you understand concepts from the Physical AI & Humanoid Robotics course.
        Ask me anything about ROS 2, simulation, NVIDIA Isaac, or VLA systems!
      </div>
    </div>
  );
}

export default function MessageList({ messages, isLoading, language }: MessageListProps) {
  const messagesEndRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, isLoading]);

  if (messages.length === 0 && !isLoading) {
    return (
      <div className={styles.messageList}>
        <EmptyState />
      </div>
    );
  }

  return (
    <div className={styles.messageList}>
      {messages.map((message) => (
        <MessageItem key={message.id} message={message} language={language} />
      ))}
      {isLoading && <TypingIndicator />}
      <div ref={messagesEndRef} />
    </div>
  );
}
