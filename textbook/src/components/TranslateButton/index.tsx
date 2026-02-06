import React, { useState, useCallback, useEffect } from "react";
import translationService from "../../services/translationService";
import styles from "./styles.module.css";

interface TranslateButtonProps {
  chapterSlug?: string;
}

// Inject Noto Nastaliq Urdu font
function injectUrduFont() {
  const fontId = "noto-nastaliq-urdu-font";
  if (!document.getElementById(fontId)) {
    const link = document.createElement("link");
    link.id = fontId;
    link.rel = "stylesheet";
    link.href =
      "https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu:wght@400;500;600;700&display=swap";
    document.head.appendChild(link);
  }
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

  // Load the Urdu font on component mount
  useEffect(() => {
    injectUrduFont();
  }, []);

  const getContentElement = useCallback(() => {
    // Find the main content container in Docusaurus - try multiple selectors
    const selectors = [
      ".theme-doc-markdown",
      "article .markdown",
      "article",
      ".docMainContainer article",
      '[class*="docItemContainer"] article',
      ".container article",
      "main article",
      ".markdown",
      "main .container",
      "main",
    ];

    for (const selector of selectors) {
      const el = document.querySelector(selector);
      if (el && el.textContent && el.textContent.trim().length > 100) {
        return el;
      }
    }
    return null;
  }, []);

  const handleTranslate = async () => {
    const contentEl = getContentElement();
    if (!contentEl) {
      setError("Could not find content to translate");
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      if (isTranslated && originalContent) {
        // Restore original content
        contentEl.innerHTML = originalContent;
        contentEl.classList.remove(styles.translatedContent);
        (contentEl as HTMLElement).style.cssText = "";
        setIsTranslated(false);
        setIsCached(false);
      } else {
        // Save original content
        setOriginalContent(contentEl.innerHTML);

        // Get text content for translation
        const textContent = contentEl.innerText;

        // Translate
        const response = await translationService.translate(
          textContent,
          "ur",
          true,
        );

        // Apply translation with inline styles for Noto Nastaliq Urdu font
        const urduFontStyle =
          "font-family: 'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', serif; direction: rtl; text-align: right; line-height: 2.4; font-size: 1.2rem;";
        const translatedHtml = response.translation
          .split("\n")
          .filter((line) => line.trim())
          .map((line) => `<p style="${urduFontStyle}">${line}</p>`)
          .join("");

        contentEl.innerHTML = translatedHtml;
        contentEl.classList.add(styles.translatedContent);
        (contentEl as HTMLElement).style.cssText = urduFontStyle;
        setIsTranslated(true);
        setIsCached(response.cached);
      }
    } catch (err) {
      console.error("Translation error:", err);
      setError("Translation failed. Please try again.");
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.translateContainer}>
      <button
        className={`${styles.translateButton} ${isTranslated ? styles.active : ""}`}
        onClick={handleTranslate}
        disabled={isLoading}
        title={isTranslated ? "Show original English" : "Translate to Urdu"}
      >
        {isLoading ? <span className={styles.spinner} /> : <TranslateIcon />}
        {isTranslated ? "English" : "اردو میں ترجمہ"}
      </button>

      {error && <div className={styles.errorMessage}>{error}</div>}
    </div>
  );
}
