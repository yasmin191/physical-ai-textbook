import React, { useState, useCallback } from "react";
import styles from "./styles.module.css";

interface UserBackground {
  programmingExperience: string;
  programmingLanguages: string[];
  rosExperience: string;
  aiMlExperience: string;
  roboticsExperience: string;
  hardwarePlatforms: string[];
  hasJetson: boolean;
  hasRobot: boolean;
  learningGoals: string[];
  preferredPace: string;
}

const CURRENT_USER_KEY = "physical_ai_current_user";

export default function PersonalizeButton() {
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [originalContent, setOriginalContent] = useState<string | null>(null);

  const getContentElement = useCallback(() => {
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

  const getUserBackground = (): UserBackground | null => {
    try {
      const bg = localStorage.getItem("userBackground");
      return bg ? JSON.parse(bg) : null;
    } catch {
      return null;
    }
  };

  const isLoggedIn = (): boolean => {
    return !!localStorage.getItem(CURRENT_USER_KEY);
  };

  const generatePersonalizedIntro = (background: UserBackground): string => {
    const parts: string[] = [];

    // Programming level intro
    if (background.programmingExperience === "none") {
      parts.push(
        `<div class="${styles.personalizedNote}">
          <strong>📚 Beginner-Friendly Guide:</strong> Since you're new to programming, we'll explain concepts step-by-step with simple examples. Don't worry if some terms are unfamiliar - we'll define everything as we go!
        </div>`
      );
    } else if (background.programmingExperience === "advanced") {
      parts.push(
        `<div class="${styles.personalizedNote}">
          <strong>⚡ Advanced Track:</strong> As an experienced programmer, you can skip the basic explanations. Focus on the robotics-specific implementations and optimization techniques highlighted in this chapter.
        </div>`
      );
    }

    // Language-specific tips
    if (background.programmingLanguages.includes("Python")) {
      parts.push(
        `<div class="${styles.personalizedTip}">
          <strong>🐍 Python User:</strong> Great news! Most code examples in this textbook use Python. You'll feel right at home with the ROS 2 rclpy library.
        </div>`
      );
    }
    if (background.programmingLanguages.includes("C++") && !background.programmingLanguages.includes("Python")) {
      parts.push(
        `<div class="${styles.personalizedTip}">
          <strong>💻 C++ User:</strong> While examples are in Python, ROS 2 also supports C++ (rclcpp). The concepts transfer directly - consider exploring the C++ equivalents for performance-critical applications.
        </div>`
      );
    }

    // ROS experience
    if (background.rosExperience === "none") {
      parts.push(
        `<div class="${styles.personalizedNote}">
          <strong>🤖 New to ROS:</strong> We'll introduce ROS 2 concepts gradually. Pay special attention to the "Key Concepts" boxes throughout this chapter.
        </div>`
      );
    } else if (background.rosExperience === "advanced") {
      parts.push(
        `<div class="${styles.personalizedTip}">
          <strong>🎯 ROS Expert:</strong> You can skim the ROS basics and focus on the humanoid-specific implementations and advanced integration patterns.
        </div>`
      );
    }

    // Hardware access
    if (background.hasJetson) {
      parts.push(
        `<div class="${styles.personalizedTip}">
          <strong>🖥️ Jetson User:</strong> Look for the "Jetson Optimization" sections - these contain hardware-specific tips for running code efficiently on your device.
        </div>`
      );
    }

    if (background.hasRobot) {
      parts.push(
        `<div class="${styles.personalizedTip}">
          <strong>🦾 Robot Access:</strong> You can test the examples on real hardware! Check the "Hands-On" exercises at the end of each section.
        </div>`
      );
    } else {
      parts.push(
        `<div class="${styles.personalizedNote}">
          <strong>💡 Simulation Mode:</strong> No robot? No problem! All examples can be run in Gazebo simulation. We'll guide you through the setup.
        </div>`
      );
    }

    // Learning pace
    if (background.preferredPace === "slow") {
      parts.push(
        `<div class="${styles.personalizedNote}">
          <strong>📖 Detailed Learning:</strong> Take your time with each section. We recommend completing all exercises before moving on.
        </div>`
      );
    } else if (background.preferredPace === "fast") {
      parts.push(
        `<div class="${styles.personalizedTip}">
          <strong>🚀 Fast Track:</strong> Focus on the highlighted key points and code examples. The detailed explanations are there if you need them.
        </div>`
      );
    }

    // Learning goals
    if (background.learningGoals.includes("Build humanoid robots")) {
      parts.push(
        `<div class="${styles.personalizedTip}">
          <strong>🎯 Your Goal - Humanoid Robots:</strong> This chapter directly contributes to your goal! Pay attention to the bipedal locomotion and balance control concepts.
        </div>`
      );
    }
    if (background.learningGoals.includes("Career in robotics")) {
      parts.push(
        `<div class="${styles.personalizedTip}">
          <strong>💼 Career Focus:</strong> Industry insights and best practices are marked with a briefcase icon throughout this chapter.
        </div>`
      );
    }

    if (parts.length === 0) {
      parts.push(
        `<div class="${styles.personalizedNote}">
          <strong>✨ Personalized for You:</strong> This content has been tailored based on your background. Enjoy your learning journey!
        </div>`
      );
    }

    return `<div class="${styles.personalizedSection}">
      <h3 class="${styles.personalizedHeader}">📋 Personalized Learning Guide</h3>
      ${parts.join("\n")}
    </div>`;
  };

  const handlePersonalize = async () => {
    if (!isLoggedIn()) {
      setError("Please sign in to personalize content");
      return;
    }

    const background = getUserBackground();
    if (!background) {
      setError("Please complete your background questionnaire first");
      return;
    }

    const contentEl = getContentElement();
    if (!contentEl) {
      setError("Could not find content to personalize");
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      if (isPersonalized && originalContent) {
        // Restore original content
        contentEl.innerHTML = originalContent;
        setIsPersonalized(false);
      } else {
        // Save original content
        setOriginalContent(contentEl.innerHTML);

        // Generate personalized intro based on user background
        const personalizedIntro = generatePersonalizedIntro(background);

        // Insert personalized content at the beginning
        contentEl.innerHTML = personalizedIntro + contentEl.innerHTML;
        setIsPersonalized(true);
      }
    } catch (err) {
      console.error("Personalization error:", err);
      setError("Personalization failed. Please try again.");
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.personalizeContainer}>
      <button
        className={`${styles.personalizeButton} ${isPersonalized ? styles.active : ""}`}
        onClick={handlePersonalize}
        disabled={isLoading}
        title={isPersonalized ? "Remove personalization" : "Personalize content based on your background"}
      >
        {isLoading ? (
          <span className={styles.spinner} />
        ) : (
          <span className={styles.icon}>✨</span>
        )}
        {isPersonalized ? "Original" : "Personalize"}
      </button>
      {error && <div className={styles.errorMessage}>{error}</div>}
    </div>
  );
}
