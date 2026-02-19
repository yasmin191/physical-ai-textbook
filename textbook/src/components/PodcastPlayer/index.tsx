import React, { useState, useEffect, useRef, useCallback } from "react";
import translationService from "../../services/translationService";
import styles from "./styles.module.css";

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
  const sentences = text.match(/[^.!?؟।]+[.!?؟।]+[\s]*/g) || [text];
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

  useEffect(() => {
    speedRef.current = playbackSpeed;
  }, [playbackSpeed]);

  useEffect(() => {
    langRef.current = language;
  }, [language]);

  // Load voices on mount
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

  useEffect(() => {
    return () => {
      window.speechSynthesis.cancel();
    };
  }, []);

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
        setIsPlaying(false);
        setIsPaused(false);
        setProgress(100);
        setStatusText("Finished");
        return;
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
          setError(
            "Speech synthesis error. Try using Chrome for best voice support.",
          );
          setIsPlaying(false);
          setIsPaused(false);
        }
      };

      window.speechSynthesis.speak(utterance);
    },
    [findVoice],
  );

  const handlePlay = useCallback(async () => {
    if (isPaused) {
      window.speechSynthesis.resume();
      setIsPaused(false);
      setIsPlaying(true);
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
          const result = await translationService.translate(text, "ur", false);
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

      window.speechSynthesis.cancel();
      speakChunk(0);
    } catch (err) {
      console.error("Play error:", err);
      setError("Something went wrong. Please try again.");
      setIsLoading(false);
      setStatusText("Ready");
    }
  }, [isPaused, speakChunk]);

  const handlePause = useCallback(() => {
    window.speechSynthesis.pause();
    setIsPaused(true);
    setIsPlaying(false);
    setStatusText("Paused");
  }, []);

  const handleStop = useCallback(() => {
    window.speechSynthesis.cancel();
    setIsPlaying(false);
    setIsPaused(false);
    setProgress(0);
    setCurrentChunkIndex(0);
    currentIndexRef.current = 0;
    setStatusText("Ready");
  }, []);

  const handleLanguageChange = useCallback(
    (newLang: "en" | "ur") => {
      if (newLang === language) return;
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
      window.speechSynthesis.cancel();
      setTimeout(() => {
        speakChunk(currentIndexRef.current);
      }, 50);
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
