import React, { useState, useRef, useEffect } from "react";
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

function HeadphonesIcon() {
  return (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <path d="M3 18v-6a9 9 0 0118 0v6" />
      <path d="M21 19a2 2 0 01-2 2h-1a2 2 0 01-2-2v-3a2 2 0 012-2h3v5zM3 19a2 2 0 002 2h1a2 2 0 002-2v-3a2 2 0 00-2-2H3v5z" />
    </svg>
  );
}

function DownloadIcon() {
  return (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <path d="M21 15v4a2 2 0 01-2 2H5a2 2 0 01-2-2v-4M7 10l5 5 5-5M12 15V3" />
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

function formatTime(seconds: number): string {
  const mins = Math.floor(seconds / 60);
  const secs = Math.floor(seconds % 60);
  return `${mins}:${secs.toString().padStart(2, "0")}`;
}

export default function PodcastPlayer({
  chapterSlug,
  chapterTitle,
}: PodcastPlayerProps) {
  const audioRef = useRef<HTMLAudioElement>(null);
  const [isPlaying, setIsPlaying] = useState(false);
  const [currentTime, setCurrentTime] = useState(0);
  const [duration, setDuration] = useState(0);
  const [playbackSpeed, setPlaybackSpeed] = useState(1);
  const [language, setLanguage] = useState<"en" | "ur">("en");
  const [audioAvailable, setAudioAvailable] = useState(true);
  const [isLoggedIn, setIsLoggedIn] = useState(false);

  // Check auth state on mount and listen for changes
  useEffect(() => {
    const checkAuth = () => {
      setIsLoggedIn(!!localStorage.getItem(CURRENT_USER_KEY));
    };
    checkAuth();

    window.addEventListener("storage", checkAuth);
    return () => window.removeEventListener("storage", checkAuth);
  }, []);

  // Audio file paths
  const audioPath = `/audio/${language}/${chapterSlug}.mp3`;

  useEffect(() => {
    const audio = audioRef.current;
    if (!audio) return;

    const handleTimeUpdate = () => setCurrentTime(audio.currentTime);
    const handleDurationChange = () => setDuration(audio.duration);
    const handleEnded = () => setIsPlaying(false);
    const handleError = () => setAudioAvailable(false);
    const handleCanPlay = () => setAudioAvailable(true);

    audio.addEventListener("timeupdate", handleTimeUpdate);
    audio.addEventListener("durationchange", handleDurationChange);
    audio.addEventListener("ended", handleEnded);
    audio.addEventListener("error", handleError);
    audio.addEventListener("canplay", handleCanPlay);

    return () => {
      audio.removeEventListener("timeupdate", handleTimeUpdate);
      audio.removeEventListener("durationchange", handleDurationChange);
      audio.removeEventListener("ended", handleEnded);
      audio.removeEventListener("error", handleError);
      audio.removeEventListener("canplay", handleCanPlay);
    };
  }, []);

  useEffect(() => {
    const audio = audioRef.current;
    if (audio) {
      audio.playbackRate = playbackSpeed;
    }
  }, [playbackSpeed]);

  useEffect(() => {
    // Reset when language changes
    const audio = audioRef.current;
    if (audio) {
      audio.pause();
      setIsPlaying(false);
      setCurrentTime(0);
      audio.load();
    }
  }, [language, chapterSlug]);

  const togglePlay = () => {
    const audio = audioRef.current;
    if (!audio) return;

    if (isPlaying) {
      audio.pause();
    } else {
      audio.play().catch(console.error);
    }
    setIsPlaying(!isPlaying);
  };

  const handleProgressClick = (e: React.MouseEvent<HTMLDivElement>) => {
    const audio = audioRef.current;
    if (!audio || !duration) return;

    const rect = e.currentTarget.getBoundingClientRect();
    const percent = (e.clientX - rect.left) / rect.width;
    audio.currentTime = percent * duration;
  };

  const cycleSpeed = () => {
    const currentIndex = PLAYBACK_SPEEDS.indexOf(playbackSpeed);
    const nextIndex = (currentIndex + 1) % PLAYBACK_SPEEDS.length;
    setPlaybackSpeed(PLAYBACK_SPEEDS[nextIndex]);
  };

  const handleDownload = () => {
    const link = document.createElement("a");
    link.href = audioPath;
    link.download = `${chapterSlug}-${language}.mp3`;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
  };

  const progress = duration > 0 ? (currentTime / duration) * 100 : 0;

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
      <audio ref={audioRef} src={audioPath} preload="metadata" />

      <div className={styles.playerHeader}>
        <div className={styles.playerTitle}>
          <HeadphonesIcon />
          <span>Listen to this chapter</span>
        </div>
        <div className={styles.languageToggle}>
          <button
            className={`${styles.langButton} ${language === "en" ? styles.active : ""}`}
            onClick={() => setLanguage("en")}
          >
            English
          </button>
          <button
            className={`${styles.langButton} ${language === "ur" ? styles.active : ""}`}
            onClick={() => setLanguage("ur")}
          >
            اردو
          </button>
        </div>
      </div>

      {audioAvailable ? (
        <div className={styles.controls}>
          <button className={styles.playButton} onClick={togglePlay}>
            {isPlaying ? <PauseIcon /> : <PlayIcon />}
          </button>

          <div className={styles.progressContainer}>
            <div className={styles.progressBar} onClick={handleProgressClick}>
              <div
                className={styles.progressFill}
                style={{ width: `${progress}%` }}
              />
            </div>
            <div className={styles.timeDisplay}>
              <span>{formatTime(currentTime)}</span>
              <span>{formatTime(duration)}</span>
            </div>
          </div>

          <div className={styles.rightControls}>
            <button className={styles.speedButton} onClick={cycleSpeed}>
              {playbackSpeed}x
            </button>
            <button
              className={styles.downloadButton}
              onClick={handleDownload}
              title="Download audio"
            >
              <DownloadIcon />
            </button>
          </div>
        </div>
      ) : (
        <div className={styles.noAudio}>
          Audio not available for this chapter yet.
        </div>
      )}
    </div>
  );
}
