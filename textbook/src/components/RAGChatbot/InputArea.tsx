import React, { useState, KeyboardEvent } from 'react';
import styles from './styles.module.css';

interface InputAreaProps {
  onSend: (message: string) => void;
  isLoading: boolean;
  selectedText: string | null;
  onClearSelection: () => void;
  placeholder?: string;
}

function SendIcon() {
  return (
    <svg viewBox="0 0 24 24" fill="currentColor">
      <path d="M2.01 21L23 12 2.01 3 2 10l15 2-15 2z" />
    </svg>
  );
}

export default function InputArea({
  onSend,
  isLoading,
  selectedText,
  onClearSelection,
  placeholder = "Ask about the textbook...",
}: InputAreaProps) {
  const [input, setInput] = useState('');

  const handleSend = () => {
    const trimmed = input.trim();
    if (trimmed && !isLoading) {
      onSend(trimmed);
      setInput('');
    }
  };

  const handleKeyDown = (e: KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  return (
    <div>
      {selectedText && (
        <div className={styles.selectedTextBadge}>
          <span className={styles.selectedTextContent}>
            📝 "{selectedText.substring(0, 50)}..."
          </span>
          <button
            className={styles.clearSelection}
            onClick={onClearSelection}
            title="Clear selection"
          >
            ×
          </button>
        </div>
      )}
      <div className={styles.inputArea}>
        <input
          type="text"
          className={styles.input}
          value={input}
          onChange={(e) => setInput(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder={selectedText ? "Ask about the selected text..." : placeholder}
          disabled={isLoading}
        />
        <button
          className={styles.sendButton}
          onClick={handleSend}
          disabled={!input.trim() || isLoading}
          title="Send message"
        >
          <SendIcon />
        </button>
      </div>
    </div>
  );
}
