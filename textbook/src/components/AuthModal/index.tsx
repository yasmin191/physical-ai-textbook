import React, { useState } from "react";
import styles from "./styles.module.css";

interface AuthModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSuccess: () => void;
}

type AuthMode = "signin" | "signup" | "background";

interface BackgroundData {
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

const AUTH_SERVER_URL =
  typeof window !== "undefined" && window.location.hostname !== "localhost"
    ? "https://physical-ai-auth.vercel.app" // Update when deployed
    : "http://localhost:3001";

export default function AuthModal({
  isOpen,
  onClose,
  onSuccess,
}: AuthModalProps) {
  const [mode, setMode] = useState<AuthMode>("signin");
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [name, setName] = useState("");
  const [error, setError] = useState("");
  const [loading, setLoading] = useState(false);

  // Background questionnaire state
  const [background, setBackground] = useState<BackgroundData>({
    programmingExperience: "beginner",
    programmingLanguages: [],
    rosExperience: "none",
    aiMlExperience: "none",
    roboticsExperience: "none",
    hardwarePlatforms: [],
    hasJetson: false,
    hasRobot: false,
    learningGoals: [],
    preferredPace: "normal",
  });

  if (!isOpen) return null;

  const handleSignIn = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError("");

    try {
      const response = await fetch(`${AUTH_SERVER_URL}/api/auth/sign-in/email`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        credentials: "include",
        body: JSON.stringify({ email, password }),
      });

      const data = await response.json();

      if (response.ok) {
        localStorage.setItem("user", JSON.stringify(data.user));
        onSuccess();
        onClose();
      } else {
        setError(data.message || "Sign in failed");
      }
    } catch (err) {
      setError("Connection error. Please try again.");
    } finally {
      setLoading(false);
    }
  };

  const handleSignUp = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError("");

    try {
      const response = await fetch(`${AUTH_SERVER_URL}/api/auth/sign-up/email`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        credentials: "include",
        body: JSON.stringify({ email, password, name }),
      });

      const data = await response.json();

      if (response.ok) {
        // After signup, show background questionnaire
        localStorage.setItem("user", JSON.stringify(data.user));
        setMode("background");
      } else {
        setError(data.message || "Sign up failed");
      }
    } catch (err) {
      setError("Connection error. Please try again.");
    } finally {
      setLoading(false);
    }
  };

  const handleBackgroundSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);

    try {
      const response = await fetch(`${AUTH_SERVER_URL}/api/user/background`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        credentials: "include",
        body: JSON.stringify(background),
      });

      if (response.ok) {
        // Save background to localStorage for personalization
        localStorage.setItem("userBackground", JSON.stringify(background));
        onSuccess();
        onClose();
      }
    } catch (err) {
      // Save locally even if server fails
      localStorage.setItem("userBackground", JSON.stringify(background));
      onSuccess();
      onClose();
    } finally {
      setLoading(false);
    }
  };

  const toggleArrayItem = (
    field: keyof BackgroundData,
    item: string
  ) => {
    const current = background[field] as string[];
    if (current.includes(item)) {
      setBackground({
        ...background,
        [field]: current.filter((i) => i !== item),
      });
    } else {
      setBackground({
        ...background,
        [field]: [...current, item],
      });
    }
  };

  const renderSignIn = () => (
    <form onSubmit={handleSignIn} className={styles.form}>
      <h2 className={styles.title}>Sign In</h2>
      <p className={styles.subtitle}>
        Welcome back! Sign in to access personalized content.
      </p>

      {error && <div className={styles.error}>{error}</div>}

      <div className={styles.field}>
        <label htmlFor="email">Email</label>
        <input
          id="email"
          type="email"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          placeholder="your@email.com"
          required
        />
      </div>

      <div className={styles.field}>
        <label htmlFor="password">Password</label>
        <input
          id="password"
          type="password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          placeholder="Your password"
          required
        />
      </div>

      <button type="submit" className={styles.submitBtn} disabled={loading}>
        {loading ? "Signing in..." : "Sign In"}
      </button>

      <p className={styles.switchMode}>
        Don't have an account?{" "}
        <button type="button" onClick={() => setMode("signup")}>
          Sign Up
        </button>
      </p>
    </form>
  );

  const renderSignUp = () => (
    <form onSubmit={handleSignUp} className={styles.form}>
      <h2 className={styles.title}>Create Account</h2>
      <p className={styles.subtitle}>
        Join us to get personalized learning content.
      </p>

      {error && <div className={styles.error}>{error}</div>}

      <div className={styles.field}>
        <label htmlFor="name">Name</label>
        <input
          id="name"
          type="text"
          value={name}
          onChange={(e) => setName(e.target.value)}
          placeholder="Your name"
          required
        />
      </div>

      <div className={styles.field}>
        <label htmlFor="signup-email">Email</label>
        <input
          id="signup-email"
          type="email"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          placeholder="your@email.com"
          required
        />
      </div>

      <div className={styles.field}>
        <label htmlFor="signup-password">Password</label>
        <input
          id="signup-password"
          type="password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          placeholder="Create a password (min 8 characters)"
          minLength={8}
          required
        />
      </div>

      <button type="submit" className={styles.submitBtn} disabled={loading}>
        {loading ? "Creating account..." : "Sign Up"}
      </button>

      <p className={styles.switchMode}>
        Already have an account?{" "}
        <button type="button" onClick={() => setMode("signin")}>
          Sign In
        </button>
      </p>
    </form>
  );

  const renderBackgroundQuestionnaire = () => (
    <form onSubmit={handleBackgroundSubmit} className={styles.form}>
      <h2 className={styles.title}>Tell Us About Yourself</h2>
      <p className={styles.subtitle}>
        Help us personalize your learning experience.
      </p>

      <div className={styles.section}>
        <h3>Software Background</h3>

        <div className={styles.field}>
          <label>Programming Experience</label>
          <select
            value={background.programmingExperience}
            onChange={(e) =>
              setBackground({ ...background, programmingExperience: e.target.value })
            }
          >
            <option value="none">No experience</option>
            <option value="beginner">Beginner (less than 1 year)</option>
            <option value="intermediate">Intermediate (1-3 years)</option>
            <option value="advanced">Advanced (3+ years)</option>
          </select>
        </div>

        <div className={styles.field}>
          <label>Programming Languages (select all that apply)</label>
          <div className={styles.checkboxGroup}>
            {["Python", "C++", "JavaScript", "Java", "Rust", "Go"].map((lang) => (
              <label key={lang} className={styles.checkbox}>
                <input
                  type="checkbox"
                  checked={background.programmingLanguages.includes(lang)}
                  onChange={() => toggleArrayItem("programmingLanguages", lang)}
                />
                {lang}
              </label>
            ))}
          </div>
        </div>

        <div className={styles.field}>
          <label>ROS Experience</label>
          <select
            value={background.rosExperience}
            onChange={(e) =>
              setBackground({ ...background, rosExperience: e.target.value })
            }
          >
            <option value="none">No experience</option>
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>

        <div className={styles.field}>
          <label>AI/ML Experience</label>
          <select
            value={background.aiMlExperience}
            onChange={(e) =>
              setBackground({ ...background, aiMlExperience: e.target.value })
            }
          >
            <option value="none">No experience</option>
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>
      </div>

      <div className={styles.section}>
        <h3>Hardware Background</h3>

        <div className={styles.field}>
          <label>Robotics/Hardware Experience</label>
          <select
            value={background.roboticsExperience}
            onChange={(e) =>
              setBackground({ ...background, roboticsExperience: e.target.value })
            }
          >
            <option value="none">No experience</option>
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>

        <div className={styles.field}>
          <label>Hardware Platforms (select all that apply)</label>
          <div className={styles.checkboxGroup}>
            {["Arduino", "Raspberry Pi", "NVIDIA Jetson", "ESP32", "STM32", "ROS Robots"].map(
              (platform) => (
                <label key={platform} className={styles.checkbox}>
                  <input
                    type="checkbox"
                    checked={background.hardwarePlatforms.includes(platform)}
                    onChange={() => toggleArrayItem("hardwarePlatforms", platform)}
                  />
                  {platform}
                </label>
              )
            )}
          </div>
        </div>

        <div className={styles.field}>
          <label className={styles.checkbox}>
            <input
              type="checkbox"
              checked={background.hasJetson}
              onChange={(e) =>
                setBackground({ ...background, hasJetson: e.target.checked })
              }
            />
            I have access to NVIDIA Jetson hardware
          </label>
        </div>

        <div className={styles.field}>
          <label className={styles.checkbox}>
            <input
              type="checkbox"
              checked={background.hasRobot}
              onChange={(e) =>
                setBackground({ ...background, hasRobot: e.target.checked })
              }
            />
            I have access to a robot (real or simulated)
          </label>
        </div>
      </div>

      <div className={styles.section}>
        <h3>Learning Preferences</h3>

        <div className={styles.field}>
          <label>Learning Goals (select all that apply)</label>
          <div className={styles.checkboxGroup}>
            {[
              "Build humanoid robots",
              "Learn ROS 2",
              "Understand AI for robotics",
              "Simulation development",
              "Career in robotics",
              "Research purposes",
            ].map((goal) => (
              <label key={goal} className={styles.checkbox}>
                <input
                  type="checkbox"
                  checked={background.learningGoals.includes(goal)}
                  onChange={() => toggleArrayItem("learningGoals", goal)}
                />
                {goal}
              </label>
            ))}
          </div>
        </div>

        <div className={styles.field}>
          <label>Preferred Learning Pace</label>
          <select
            value={background.preferredPace}
            onChange={(e) =>
              setBackground({ ...background, preferredPace: e.target.value })
            }
          >
            <option value="slow">Slow (detailed explanations)</option>
            <option value="normal">Normal</option>
            <option value="fast">Fast (quick overview)</option>
          </select>
        </div>
      </div>

      <button type="submit" className={styles.submitBtn} disabled={loading}>
        {loading ? "Saving..." : "Complete Setup"}
      </button>

      <button
        type="button"
        className={styles.skipBtn}
        onClick={() => {
          localStorage.setItem("userBackground", JSON.stringify(background));
          onSuccess();
          onClose();
        }}
      >
        Skip for now
      </button>
    </form>
  );

  return (
    <div className={styles.overlay} onClick={onClose}>
      <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
        <button className={styles.closeBtn} onClick={onClose}>
          &times;
        </button>
        {mode === "signin" && renderSignIn()}
        {mode === "signup" && renderSignUp()}
        {mode === "background" && renderBackgroundQuestionnaire()}
      </div>
    </div>
  );
}
