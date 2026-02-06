/**
 * Translation service - Client-side translation using Google Translate API
 * For production, consider using a paid API or deploying a backend
 */

export interface TranslateResponse {
  translation: string;
  source_language: string;
  target_language: string;
  cached: boolean;
}

// Local storage cache for translations
const CACHE_PREFIX = "translation_cache_";
const CACHE_EXPIRY = 7 * 24 * 60 * 60 * 1000; // 7 days

interface CacheEntry {
  translation: string;
  timestamp: number;
}

class TranslationService {
  private getCacheKey(content: string, targetLang: string): string {
    // Simple hash for cache key
    const hash = content.split("").reduce((a, b) => {
      a = (a << 5) - a + b.charCodeAt(0);
      return a & a;
    }, 0);
    return `${CACHE_PREFIX}${targetLang}_${Math.abs(hash)}`;
  }

  private getFromCache(content: string, targetLang: string): string | null {
    try {
      const key = this.getCacheKey(content, targetLang);
      const cached = localStorage.getItem(key);

      if (cached) {
        const entry: CacheEntry = JSON.parse(cached);
        if (Date.now() - entry.timestamp < CACHE_EXPIRY) {
          return entry.translation;
        }
        // Remove expired cache
        localStorage.removeItem(key);
      }
    } catch (error) {
      console.error("Cache read error:", error);
    }
    return null;
  }

  private saveToCache(
    content: string,
    targetLang: string,
    translation: string,
  ): void {
    try {
      const key = this.getCacheKey(content, targetLang);
      const entry: CacheEntry = {
        translation,
        timestamp: Date.now(),
      };
      localStorage.setItem(key, JSON.stringify(entry));
    } catch (error) {
      console.error("Cache write error:", error);
    }
  }

  /**
   * Translate using MyMemory free translation API
   * Free tier: 5000 chars/day, no API key required
   */
  private async translateWithMyMemory(
    text: string,
    targetLang: string,
  ): Promise<string> {
    // Split text into chunks (MyMemory has 500 char limit per request)
    const chunks = this.splitIntoChunks(text, 450);
    const translatedChunks: string[] = [];

    for (const chunk of chunks) {
      if (!chunk.trim()) {
        translatedChunks.push("");
        continue;
      }

      const url = `https://api.mymemory.translated.net/get?q=${encodeURIComponent(chunk)}&langpair=en|${targetLang}`;

      try {
        const response = await fetch(url);
        const data = await response.json();

        if (data.responseStatus === 200 && data.responseData?.translatedText) {
          translatedChunks.push(data.responseData.translatedText);
        } else {
          // If translation fails, return original
          translatedChunks.push(chunk);
        }
      } catch (error) {
        console.error("Translation chunk error:", error);
        translatedChunks.push(chunk);
      }

      // Small delay to avoid rate limiting
      await new Promise((resolve) => setTimeout(resolve, 100));
    }

    return translatedChunks.join(" ");
  }

  private splitIntoChunks(text: string, maxLength: number): string[] {
    const chunks: string[] = [];
    const sentences = text.split(/(?<=[.!?।])\s+/);
    let currentChunk = "";

    for (const sentence of sentences) {
      if (currentChunk.length + sentence.length > maxLength) {
        if (currentChunk) {
          chunks.push(currentChunk.trim());
        }
        // If single sentence is too long, split by words
        if (sentence.length > maxLength) {
          const words = sentence.split(" ");
          currentChunk = "";
          for (const word of words) {
            if (currentChunk.length + word.length > maxLength) {
              chunks.push(currentChunk.trim());
              currentChunk = word + " ";
            } else {
              currentChunk += word + " ";
            }
          }
        } else {
          currentChunk = sentence + " ";
        }
      } else {
        currentChunk += sentence + " ";
      }
    }

    if (currentChunk.trim()) {
      chunks.push(currentChunk.trim());
    }

    return chunks;
  }

  async translate(
    content: string,
    targetLanguage: string = "ur",
    preserveCode: boolean = true,
  ): Promise<TranslateResponse> {
    // Check local cache first
    const cached = this.getFromCache(content, targetLanguage);
    if (cached) {
      return {
        translation: cached,
        source_language: "en",
        target_language: targetLanguage,
        cached: true,
      };
    }

    // Extract and preserve code blocks if needed
    let textToTranslate = content;
    const codeBlocks: { placeholder: string; code: string }[] = [];

    if (preserveCode) {
      // Replace code blocks with placeholders
      let codeIndex = 0;
      textToTranslate = content.replace(/```[\s\S]*?```|`[^`]+`/g, (match) => {
        const placeholder = `__CODE_BLOCK_${codeIndex}__`;
        codeBlocks.push({ placeholder, code: match });
        codeIndex++;
        return placeholder;
      });
    }

    // Translate using MyMemory API
    let translation = await this.translateWithMyMemory(
      textToTranslate,
      targetLanguage,
    );

    // Restore code blocks
    for (const { placeholder, code } of codeBlocks) {
      translation = translation.replace(placeholder, code);
    }

    // Save to local cache
    this.saveToCache(content, targetLanguage, translation);

    return {
      translation,
      source_language: "en",
      target_language: targetLanguage,
      cached: false,
    };
  }

  async translateChapter(
    content: string,
    chapterSlug: string,
    targetLanguage: string = "ur",
  ): Promise<TranslateResponse> {
    return this.translate(content, targetLanguage, true);
  }

  clearLocalCache(): number {
    let count = 0;
    const keys = Object.keys(localStorage);

    for (const key of keys) {
      if (key.startsWith(CACHE_PREFIX)) {
        localStorage.removeItem(key);
        count++;
      }
    }

    return count;
  }
}

export const translationService = new TranslationService();
export default translationService;
