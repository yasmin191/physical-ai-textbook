/**
 * Translation service - Client-side translation using Google Translate free endpoint
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
   * Translate a single chunk using Google Translate free endpoint
   */
  private async translateChunk(
    text: string,
    sourceLang: string,
    targetLang: string,
  ): Promise<string> {
    const url = `https://translate.googleapis.com/translate_a/single?client=gtx&sl=${sourceLang}&tl=${targetLang}&dt=t&q=${encodeURIComponent(text)}`;

    const response = await fetch(url);
    if (!response.ok) {
      throw new Error(`Translation request failed: ${response.status}`);
    }

    const data = await response.json();

    // Response format: [[["translated text","original text",null,null,10]],null,"en"]
    if (Array.isArray(data) && Array.isArray(data[0])) {
      return data[0].map((item: any[]) => item[0]).join("");
    }

    throw new Error("Unexpected translation response format");
  }

  private splitIntoChunks(text: string, maxLength: number): string[] {
    const chunks: string[] = [];
    const sentences = text.split(/(?<=[.!?।؟])\s+/);
    let currentChunk = "";

    for (const sentence of sentences) {
      if (currentChunk.length + sentence.length > maxLength) {
        if (currentChunk) {
          chunks.push(currentChunk.trim());
        }
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
      let codeIndex = 0;
      textToTranslate = content.replace(/```[\s\S]*?```|`[^`]+`/g, (match) => {
        const placeholder = `__CODE_BLOCK_${codeIndex}__`;
        codeBlocks.push({ placeholder, code: match });
        codeIndex++;
        return placeholder;
      });
    }

    // Split into chunks (Google Translate handles ~5000 chars per request)
    const chunks = this.splitIntoChunks(textToTranslate, 4500);
    const translatedChunks: string[] = [];

    for (const chunk of chunks) {
      if (!chunk.trim()) {
        translatedChunks.push("");
        continue;
      }

      try {
        const translated = await this.translateChunk(
          chunk,
          "en",
          targetLanguage,
        );
        translatedChunks.push(translated);
      } catch (error) {
        console.error("Translation chunk error:", error);
        translatedChunks.push(chunk);
      }

      // Small delay between requests
      if (chunks.length > 1) {
        await new Promise((resolve) => setTimeout(resolve, 200));
      }
    }

    let translation = translatedChunks.join(" ");

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
