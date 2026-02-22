import React, { useState, useEffect, useRef, useCallback } from "react";
import translationService from "../../services/translationService";
import styles from "./styles.module.css";

// Fetch Google TTS audio — uses Vercel rewrite proxy in production (no CORS issues),
// falls back to allorigins public proxy for local dev.
async function fetchUrduAudioBlob(text: string): Promise<string | null> {
  const params = `ie=UTF-8&q=${encodeURIComponent(text)}&tl=ur&client=tw-ob`;

  // Primary: Vercel rewrite proxy (/api/tts forwards to translate.google.com/translate_tts)
  // Works in production with no CORS headers needed since it's same-origin.
  const urls = [
    `/api/tts?${params}`,
    `https://api.allorigins.win/raw?url=${encodeURIComponent(`https://translate.google.com/translate_tts?${params}`)}`,
  ];

  for (const url of urls) {
    try {
      const res = await fetch(url);
      if (!res.ok) continue;
      const blob = await res.blob();
      if (blob.size > 0) return URL.createObjectURL(blob);
    } catch {
      // try next
    }
  }
  return null;
}

interface PodcastPlayerProps {
  chapterSlug: string;
  chapterTitle?: string;
}

const PLAYBACK_SPEEDS = [0.5, 0.75, 1, 1.25, 1.5, 2];

function PlayIcon() {
  return (
    <svg viewBox="0 0 24 24" fill="currentColor">
      <path d="M8 5v14l11-7z" />
    </svg>
  );
}

function PauseIcon() {
  return (
    <svg viewBox="0 0 24 24" fill="currentColor">
      <path d="M6 19h4V5H6v14zm8-14v14h4V5h-4z" />
    </svg>
  );
}

function StopIcon() {
  return (
    <svg viewBox="0 0 24 24" fill="currentColor">
      <path d="M6 6h12v12H6z" />
    </svg>
  );
}

function HeadphonesIcon() {
  return (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <path d="M3 18v-6a9 9 0 0118 0v6" />
      <path d="M21 19a2 2 0 01-2 2h-1a2 2 0 01-2-2v-3a2 2 0 012-2h3v5zM3 19a2 2 0 002 2h1a2 2 0 002-2v-3a2 2 0 00-2-2H3v5z" />
    </svg>
  );
}

function LockIcon() {
  return (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <rect x="3" y="11" width="18" height="11" rx="2" ry="2" />
      <path d="M7 11V7a5 5 0 0110 0v4" />
    </svg>
  );
}

const CURRENT_USER_KEY = "physical_ai_current_user";

function splitTextIntoChunks(text: string, maxLen = 200): string[] {
  const sentences = text.match(/[^.!?؟।۔]+[.!?؟।۔]+[\s]*/g) || [text];
  const chunks: string[] = [];
  let current = "";

  for (const sentence of sentences) {
    if (current.length + sentence.length > maxLen && current.length > 0) {
      chunks.push(current.trim());
      current = "";
    }
    current += sentence;
  }
  if (current.trim()) {
    chunks.push(current.trim());
  }
  return chunks;
}

function getPageText(): string {
  const selectors = [
    ".theme-doc-markdown",
    "article .markdown",
    "article",
    "main article",
  ];

  for (const selector of selectors) {
    const el = document.querySelector(selector);
    if (el && el.textContent && el.textContent.trim().length > 100) {
      const clone = el.cloneNode(true) as HTMLElement;
      clone
        .querySelectorAll("pre, code, .hash-link, nav, .podcastPlayer")
        .forEach((node) => node.remove());
      return clone.textContent?.trim() || "";
    }
  }
  return "";
}

// Wait for voices to be loaded (they load async in most browsers)
function getVoicesAsync(): Promise<SpeechSynthesisVoice[]> {
  return new Promise((resolve) => {
    const voices = window.speechSynthesis.getVoices();
    if (voices.length > 0) {
      resolve(voices);
      return;
    }
    // Voices not loaded yet, wait for the event
    const handler = () => {
      const v = window.speechSynthesis.getVoices();
      resolve(v);
      window.speechSynthesis.removeEventListener("voiceschanged", handler);
    };
    window.speechSynthesis.addEventListener("voiceschanged", handler);
    // Fallback timeout in case event never fires
    setTimeout(() => resolve(window.speechSynthesis.getVoices()), 1000);
  });
}

export default function PodcastPlayer({
  chapterSlug,
  chapterTitle,
}: PodcastPlayerProps) {
  const [isPlaying, setIsPlaying] = useState(false);
  const [isPaused, setIsPaused] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [playbackSpeed, setPlaybackSpeed] = useState(1);
  const [language, setLanguage] = useState<"en" | "ur">("en");
  const [isLoggedIn, setIsLoggedIn] = useState(false);
  const [progress, setProgress] = useState(0);
  const [currentChunkIndex, setCurrentChunkIndex] = useState(0);
  const [totalChunks, setTotalChunks] = useState(0);
  const [error, setError] = useState<string | null>(null);
  const [statusText, setStatusText] = useState("Ready");

  const chunksRef = useRef<string[]>([]);
  const currentIndexRef = useRef(0);
  const speedRef = useRef(1);
  const langRef = useRef<"en" | "ur">("en");
  const voicesRef = useRef<SpeechSynthesisVoice[]>([]);
  const keepAliveRef = useRef<ReturnType<typeof setInterval> | null>(null);
  // For Urdu: holds the currently playing Audio element
  const audioRef = useRef<HTMLAudioElement | null>(null);
  // Flag to cancel Urdu audio playback mid-sequence
  const urduCancelledRef = useRef(false);

  useEffect(() => {
    speedRef.current = playbackSpeed;
  }, [playbackSpeed]);

  useEffect(() => {
    langRef.current = language;
  }, [language]);

  // Load voices on mount for English TTS
  useEffect(() => {
    getVoicesAsync().then((voices) => {
      voicesRef.current = voices;
    });
  }, []);

  useEffect(() => {
    const checkAuth = () => {
      setIsLoggedIn(!!localStorage.getItem(CURRENT_USER_KEY));
    };
    checkAuth();
    window.addEventListener("storage", checkAuth);
    return () => window.removeEventListener("storage", checkAuth);
  }, []);

  // Chrome stops speechSynthesis after ~15s; keep it alive with pause/resume every 10s
  const stopKeepAlive = useCallback(() => {
    if (keepAliveRef.current) {
      clearInterval(keepAliveRef.current);
      keepAliveRef.current = null;
    }
  }, []);

  const startKeepAlive = useCallback(() => {
    stopKeepAlive();
    keepAliveRef.current = setInterval(() => {
      if (window.speechSynthesis.speaking && !window.speechSynthesis.paused) {
        window.speechSynthesis.pause();
        window.speechSynthesis.resume();
      }
    }, 10000);
  }, [stopKeepAlive]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      urduCancelledRef.current = true;
      if (audioRef.current) {
        audioRef.current.pause();
        audioRef.current = null;
      }
      stopKeepAlive();
      window.speechSynthesis.cancel();
    };
  }, [stopKeepAlive]);

  const findVoice = useCallback((lang: "en" | "ur") => {
    const voices =
      voicesRef.current.length > 0
        ? voicesRef.current
        : window.speechSynthesis.getVoices();

    if (lang === "ur") {
      return (
        voices.find((v) => v.lang.startsWith("ur")) ||
        voices.find((v) => v.lang.startsWith("hi")) ||
        voices.find((v) => v.lang.startsWith("en")) ||
        null
      );
    }
    return voices.find((v) => v.lang.startsWith("en")) || null;
  }, []);

  const speakChunk = useCallback(
    (index: number) => {
      const chunks = chunksRef.current;
      if (index >= chunks.length) {
        stopKeepAlive();
        setIsPlaying(false);
        setIsPaused(false);
        setProgress(100);
        setStatusText("Finished");
        return;
      }

      // Start keep-alive on first chunk to prevent Chrome stopping after ~15s
      if (index === 0) {
        startKeepAlive();
      }

      const utterance = new SpeechSynthesisUtterance(chunks[index]);
      utterance.rate = speedRef.current;

      const voice = findVoice(langRef.current);
      if (voice) {
        utterance.voice = voice;
        utterance.lang = voice.lang;
      } else {
        // Set lang even without a specific voice — browser will try its best
        utterance.lang = langRef.current === "ur" ? "hi-IN" : "en-US";
      }

      utterance.onstart = () => {
        setStatusText(`Part ${index + 1} of ${chunks.length}`);
      };

      utterance.onend = () => {
        const nextIndex = index + 1;
        currentIndexRef.current = nextIndex;
        setCurrentChunkIndex(nextIndex);
        setProgress(Math.round((nextIndex / chunks.length) * 100));
        speakChunk(nextIndex);
      };

      utterance.onerror = (e) => {
        if (e.error !== "canceled" && e.error !== "interrupted") {
          console.error("Speech error:", e.error);
          stopKeepAlive();
          setError(
            "Speech synthesis error. Try using Chrome for best voice support.",
          );
          setIsPlaying(false);
          setIsPaused(false);
        }
      };

      window.speechSynthesis.speak(utterance);
    },
    [findVoice, startKeepAlive, stopKeepAlive],
  );

  // Speak a single Urdu chunk: try Google TTS via CORS proxy, fall back to Web Speech API
  const speakUrduChunk = useCallback(
    (chunk: string, index: number, total: number): Promise<void> => {
      return new Promise<void>(async (resolve) => {
        if (urduCancelledRef.current) { resolve(); return; }

        // Try Google TTS via proxy first
        const blobUrl = await fetchUrduAudioBlob(chunk);

        if (urduCancelledRef.current) {
          if (blobUrl) URL.revokeObjectURL(blobUrl);
          resolve();
          return;
        }

        if (blobUrl) {
          const audio = new Audio(blobUrl);
          audio.playbackRate = speedRef.current;
          audioRef.current = audio;
          audio.onended = () => {
            URL.revokeObjectURL(blobUrl);
            setProgress(Math.round(((index + 1) / total) * 100));
            resolve();
          };
          audio.onerror = () => {
            URL.revokeObjectURL(blobUrl);
            resolve();
          };
          audio.play().catch(() => { URL.revokeObjectURL(blobUrl); resolve(); });
          return;
        }

        // Proxy failed — fall back to Web Speech API with best available voice
        const voices = voicesRef.current.length > 0
          ? voicesRef.current
          : window.speechSynthesis.getVoices();
        const voice =
          voices.find((v) => v.lang.startsWith("ur")) ||
          voices.find((v) => v.lang.startsWith("hi")) ||
          voices.find((v) => v.lang.startsWith("en")) ||
          null;

        const utterance = new SpeechSynthesisUtterance(chunk);
        utterance.rate = speedRef.current;
        if (voice) {
          utterance.voice = voice;
          utterance.lang = voice.lang;
        } else {
          utterance.lang = "hi-IN";
        }
        utterance.onend = () => {
          setProgress(Math.round(((index + 1) / total) * 100));
          resolve();
        };
        utterance.onerror = () => resolve();
        window.speechSynthesis.speak(utterance);
      });
    },
    [],
  );

  // Urdu TTS: plays chunks sequentially using Google TTS audio (proxy) with Web Speech fallback
  const speakUrduChunks = useCallback(
    async (startIndex: number) => {
      const chunks = chunksRef.current;
      urduCancelledRef.current = false;

      for (let i = startIndex; i < chunks.length; i++) {
        if (urduCancelledRef.current) break;

        // Reuse paused audio for resume case
        const existing = audioRef.current;
        if (i === startIndex && existing && existing.paused && !existing.ended) {
          existing.playbackRate = speedRef.current;
          await new Promise<void>((resolve) => {
            existing.onended = () => {
              setProgress(Math.round(((i + 1) / chunks.length) * 100));
              resolve();
            };
            existing.onerror = () => resolve();
            existing.play().catch(() => resolve());
          });
          continue;
        }

        currentIndexRef.current = i;
        setCurrentChunkIndex(i);
        setStatusText(`Part ${i + 1} of ${chunks.length}`);

        await speakUrduChunk(chunks[i], i, chunks.length);
      }

      if (!urduCancelledRef.current) {
        audioRef.current = null;
        setIsPlaying(false);
        setIsPaused(false);
        setProgress(100);
        setStatusText("Finished");
      }
    },
    [speakUrduChunk],
  );

  const handlePlay = useCallback(async () => {
    if (isPaused) {
      if (langRef.current === "ur") {
        setIsPaused(false);
        setIsPlaying(true);
        // speakUrduChunks will detect the paused audioRef and resume it,
        // then continue with subsequent chunks
        speakUrduChunks(currentIndexRef.current);
      } else {
        startKeepAlive();
        window.speechSynthesis.resume();
        setIsPaused(false);
        setIsPlaying(true);
      }
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      // Ensure voices are loaded
      if (voicesRef.current.length === 0) {
        voicesRef.current = await getVoicesAsync();
      }

      const text = getPageText();
      if (!text) {
        setError("Could not find chapter content to read");
        setIsLoading(false);
        return;
      }

      let textToSpeak = text;

      if (langRef.current === "ur") {
        setStatusText("Translating to Urdu...");
        try {
          const result = await translationService.translate(
            text,
            "ur",
            false,
            (current, total) => {
              setStatusText(`Translating... (${current} of ${total})`);
            },
          );
          textToSpeak = result.translation;
        } catch (err) {
          console.error("Translation error:", err);
          setError("Failed to translate to Urdu. Please try again.");
          setIsLoading(false);
          setStatusText("Ready");
          return;
        }
      }

      const chunks = splitTextIntoChunks(textToSpeak);
      chunksRef.current = chunks;
      currentIndexRef.current = 0;
      setTotalChunks(chunks.length);
      setCurrentChunkIndex(0);
      setProgress(0);
      setIsLoading(false);
      setIsPlaying(true);
      setIsPaused(false);
      setStatusText(`Part 1 of ${chunks.length}`);

      if (langRef.current === "ur") {
        // Urdu: use Google TTS audio (works without installing any voices)
        speakUrduChunks(0);
      } else {
        // English: use Web Speech API
        window.speechSynthesis.cancel();
        speakChunk(0);
      }
    } catch (err) {
      console.error("Play error:", err);
      setError("Something went wrong. Please try again.");
      setIsLoading(false);
      setStatusText("Ready");
    }
  }, [isPaused, speakChunk, speakUrduChunks, startKeepAlive]);

  const handlePause = useCallback(() => {
    if (langRef.current === "ur") {
      urduCancelledRef.current = true;
      if (audioRef.current) {
        audioRef.current.pause();
      }
    } else {
      stopKeepAlive();
      window.speechSynthesis.pause();
    }
    setIsPaused(true);
    setIsPlaying(false);
    setStatusText("Paused");
  }, [stopKeepAlive]);

  const handleStop = useCallback(() => {
    urduCancelledRef.current = true;
    if (audioRef.current) {
      audioRef.current.pause();
      audioRef.current = null;
    }
    stopKeepAlive();
    window.speechSynthesis.cancel();
    setIsPlaying(false);
    setIsPaused(false);
    setProgress(0);
    setCurrentChunkIndex(0);
    currentIndexRef.current = 0;
    setStatusText("Ready");
  }, [stopKeepAlive]);

  const handleLanguageChange = useCallback(
    (newLang: "en" | "ur") => {
      if (newLang === language) return;
      urduCancelledRef.current = true;
      if (audioRef.current) {
        audioRef.current.pause();
        audioRef.current = null;
      }
      window.speechSynthesis.cancel();
      setIsPlaying(false);
      setIsPaused(false);
      setProgress(0);
      setCurrentChunkIndex(0);
      currentIndexRef.current = 0;
      setLanguage(newLang);
      setStatusText("Ready");
      setError(null);
    },
    [language],
  );

  const cycleSpeed = useCallback(() => {
    const currentIndex = PLAYBACK_SPEEDS.indexOf(playbackSpeed);
    const nextIndex = (currentIndex + 1) % PLAYBACK_SPEEDS.length;
    const newSpeed = PLAYBACK_SPEEDS[nextIndex];
    setPlaybackSpeed(newSpeed);

    if (isPlaying) {
      if (langRef.current === "ur") {
        // Update the currently playing audio element's rate immediately
        if (audioRef.current) {
          audioRef.current.playbackRate = newSpeed;
        }
      } else {
        window.speechSynthesis.cancel();
        setTimeout(() => {
          speakChunk(currentIndexRef.current);
        }, 50);
      }
    }
  }, [playbackSpeed, isPlaying, speakChunk]);

  if (!isLoggedIn) {
    return (
      <div className={styles.podcastPlayer}>
        <div className={styles.playerHeader}>
          <div className={styles.playerTitle}>
            <HeadphonesIcon />
            <span>Listen to this chapter</span>
          </div>
        </div>
        <div className={styles.lockedState}>
          <LockIcon />
          <span>Sign in to access podcasts in English and Urdu</span>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.podcastPlayer}>
      <div className={styles.playerHeader}>
        <div className={styles.playerTitle}>
          <HeadphonesIcon />
          <span>Listen to this chapter</span>
        </div>
        <div className={styles.languageToggle}>
          <button
            className={`${styles.langButton} ${language === "en" ? styles.active : ""}`}
            onClick={() => handleLanguageChange("en")}
            disabled={isLoading}
          >
            English
          </button>
          <button
            className={`${styles.langButton} ${language === "ur" ? styles.active : ""}`}
            onClick={() => handleLanguageChange("ur")}
            disabled={isLoading}
          >
            اردو
          </button>
        </div>
      </div>

      <div className={styles.controls}>
        <button
          className={styles.playButton}
          onClick={isPlaying ? handlePause : handlePlay}
          disabled={isLoading}
        >
          {isLoading ? (
            <span className={styles.spinner} />
          ) : isPlaying ? (
            <PauseIcon />
          ) : (
            <PlayIcon />
          )}
        </button>

        <div className={styles.progressContainer}>
          <div className={styles.progressBar}>
            <div
              className={styles.progressFill}
              style={{ width: `${progress}%` }}
            />
          </div>
          <div className={styles.timeDisplay}>
            <span>{statusText}</span>
            <span>{progress}%</span>
          </div>
        </div>

        <div className={styles.rightControls}>
          <button className={styles.speedButton} onClick={cycleSpeed}>
            {playbackSpeed}x
          </button>
          {(isPlaying || isPaused) && (
            <button
              className={styles.stopButton}
              onClick={handleStop}
              title="Stop"
            >
              <StopIcon />
            </button>
          )}
        </div>
      </div>

      {error && <div className={styles.errorText}>{error}</div>}
    </div>
  );
}
