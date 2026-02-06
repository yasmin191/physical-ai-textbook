import React, { useState, useCallback } from 'react';
import translationService from '../../services/translationService';
import styles from './styles.module.css';

interface TranslateButtonProps {
  chapterSlug?: string;
}

function TranslateIcon() {
  return (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <path d="M3 5h12M9 3v2m1.048 9.5A18.022 18.022 0 016.412 9m6.088 9h7M11 21l5-10 5 10M12.751 5C11.783 10.77 8.07 15.61 3 18.129" />
    </svg>
  );
}

export default function TranslateButton({ chapterSlug }: TranslateButtonProps) {
  const [isTranslated, setIsTranslated] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [originalContent, setOriginalContent] = useState<string | null>(null);
  const [isCached, setIsCached] = useState(false);

  const getContentElement = useCallback(() => {
    // Find the main content container in Docusaurus
    return document.querySelector('.theme-doc-markdown') ||
           document.querySelector('article') ||
           document.querySelector('.markdown');
  }, []);

  const handleTranslate = async () => {
    const contentEl = getContentElement();
    if (!contentEl) {
      setError('Could not find content to translate');
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      if (isTranslated && originalContent) {
        // Restore original content
        contentEl.innerHTML = originalContent;
        contentEl.classList.remove(styles.translatedContent);
        setIsTranslated(false);
        setIsCached(false);
      } else {
        // Save original content
        setOriginalContent(contentEl.innerHTML);

        // Get text content for translation
        const textContent = contentEl.innerText;

        // Translate
        const response = await translationService.translate(textContent, 'ur', true);

        // Apply translation (simplified - in production, use proper markdown rendering)
        const translatedHtml = response.translation
          .split('\n')
          .map(line => `<p>${line}</p>`)
          .join('');

        contentEl.innerHTML = translatedHtml;
        contentEl.classList.add(styles.translatedContent);
        setIsTranslated(true);
        setIsCached(response.cached);
      }
    } catch (err) {
      console.error('Translation error:', err);
      setError('Translation failed. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.translateContainer}>
      <button
        className={`${styles.translateButton} ${isTranslated ? styles.active : ''}`}
        onClick={handleTranslate}
        disabled={isLoading}
        title={isTranslated ? 'Show original English' : 'Translate to Urdu'}
      >
        {isLoading ? (
          <span className={styles.spinner} />
        ) : (
          <TranslateIcon />
        )}
        {isTranslated ? 'English' : 'اردو میں ترجمہ'}
      </button>
      {isCached && isTranslated && (
        <span className={styles.statusBadge}>Cached</span>
      )}
      {error && <div className={styles.errorMessage}>{error}</div>}
    </div>
  );
}
