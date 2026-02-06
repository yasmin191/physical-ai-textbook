/**
 * Translation service for communicating with the translation API.
 */

const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-api-domain.vercel.app/api/v1'
  : 'http://localhost:8000/api/v1';

export interface TranslateResponse {
  translation: string;
  source_language: string;
  target_language: string;
  cached: boolean;
}

// Local storage cache for translations
const CACHE_PREFIX = 'translation_cache_';
const CACHE_EXPIRY = 7 * 24 * 60 * 60 * 1000; // 7 days

interface CacheEntry {
  translation: string;
  timestamp: number;
}

class TranslationService {
  private baseUrl: string;

  constructor() {
    this.baseUrl = API_BASE_URL;
  }

  private getCacheKey(content: string, targetLang: string): string {
    // Simple hash for cache key
    const hash = content.split('').reduce((a, b) => {
      a = ((a << 5) - a) + b.charCodeAt(0);
      return a & a;
    }, 0);
    return `${CACHE_PREFIX}${targetLang}_${hash}`;
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
      console.error('Cache read error:', error);
    }
    return null;
  }

  private saveToCache(content: string, targetLang: string, translation: string): void {
    try {
      const key = this.getCacheKey(content, targetLang);
      const entry: CacheEntry = {
        translation,
        timestamp: Date.now(),
      };
      localStorage.setItem(key, JSON.stringify(entry));
    } catch (error) {
      console.error('Cache write error:', error);
    }
  }

  async translate(
    content: string,
    targetLanguage: string = 'ur',
    preserveCode: boolean = true
  ): Promise<TranslateResponse> {
    // Check local cache first
    const cached = this.getFromCache(content, targetLanguage);
    if (cached) {
      return {
        translation: cached,
        source_language: 'en',
        target_language: targetLanguage,
        cached: true,
      };
    }

    // Call API
    const response = await fetch(`${this.baseUrl}/translate/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        content,
        target_language: targetLanguage,
        preserve_code: preserveCode,
        use_cache: true,
      }),
    });

    if (!response.ok) {
      throw new Error('Translation failed');
    }

    const data: TranslateResponse = await response.json();

    // Save to local cache
    this.saveToCache(content, targetLanguage, data.translation);

    return data;
  }

  async translateChapter(
    content: string,
    chapterSlug: string,
    targetLanguage: string = 'ur'
  ): Promise<TranslateResponse> {
    // Check local cache first
    const cached = this.getFromCache(content, targetLanguage);
    if (cached) {
      return {
        translation: cached,
        source_language: 'en',
        target_language: targetLanguage,
        cached: true,
      };
    }

    const response = await fetch(`${this.baseUrl}/translate/chapter`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        content,
        chapter_slug: chapterSlug,
        target_language: targetLanguage,
      }),
    });

    if (!response.ok) {
      throw new Error('Chapter translation failed');
    }

    const data: TranslateResponse = await response.json();

    // Save to local cache
    this.saveToCache(content, targetLanguage, data.translation);

    return data;
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
