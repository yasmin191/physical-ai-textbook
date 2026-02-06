import React from 'react';
import MessageList, { Message } from './MessageList';
import InputArea from './InputArea';
import styles from './styles.module.css';

interface ChatWindowProps {
  messages: Message[];
  isLoading: boolean;
  language: string;
  selectedText: string | null;
  onSend: (message: string) => void;
  onClose: () => void;
  onClearSelection: () => void;
  onLanguageChange: (lang: string) => void;
}

function CloseIcon() {
  return (
    <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
      <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z" />
    </svg>
  );
}

function LanguageToggle({
  language,
  onChange,
}: {
  language: string;
  onChange: (lang: string) => void;
}) {
  return (
    <div className={styles.languageToggle}>
      <button
        className={`${styles.langButton} ${language === 'en' ? styles.active : ''}`}
        onClick={() => onChange('en')}
      >
        EN
      </button>
      <button
        className={`${styles.langButton} ${language === 'ur' ? styles.active : ''}`}
        onClick={() => onChange('ur')}
      >
        اردو
      </button>
    </div>
  );
}

export default function ChatWindow({
  messages,
  isLoading,
  language,
  selectedText,
  onSend,
  onClose,
  onClearSelection,
  onLanguageChange,
}: ChatWindowProps) {
  return (
    <div className={styles.chatWindow}>
      <div className={styles.chatHeader}>
        <h3 className={styles.chatTitle}>📚 Textbook Assistant</h3>
        <div style={{ display: 'flex', alignItems: 'center' }}>
          <LanguageToggle language={language} onChange={onLanguageChange} />
          <button className={styles.closeButton} onClick={onClose} title="Close chat">
            <CloseIcon />
          </button>
        </div>
      </div>
      <MessageList messages={messages} isLoading={isLoading} language={language} />
      <InputArea
        onSend={onSend}
        isLoading={isLoading}
        selectedText={selectedText}
        onClearSelection={onClearSelection}
      />
    </div>
  );
}
