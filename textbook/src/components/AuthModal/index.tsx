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

interface User {
  id: string;
  name: string;
  email: string;
  createdAt: string;
}

// Simple localStorage-based auth for hackathon demo
const USERS_KEY = "physical_ai_users";
const CURRENT_USER_KEY = "physical_ai_current_user";

function getUsers(): Record<
  string,
  { user: User; password: string; background?: BackgroundData }
> {
  try {
    const users = localStorage.getItem(USERS_KEY);
    return users ? JSON.parse(users) : {};
  } catch {
    return {};
  }
}

function saveUsers(
  users: Record<
    string,
    { user: User; password: string; background?: BackgroundData }
  >,
) {
  localStorage.setItem(USERS_KEY, JSON.stringify(users));
}

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
      const users = getUsers();
      const userEntry = users[email.toLowerCase()];

      if (!userEntry) {
        setError("No account found with this email");
        setLoading(false);
        return;
      }

      if (userEntry.password !== password) {
        setError("Incorrect password");
        setLoading(false);
        return;
      }

      localStorage.setItem(CURRENT_USER_KEY, JSON.stringify(userEntry.user));
      if (userEntry.background) {
        localStorage.setItem(
          "userBackground",
          JSON.stringify(userEntry.background),
        );
      }
      onSuccess();
      onClose();
    } catch (err) {
      setError("Sign in failed. Please try again.");
    } finally {
      setLoading(false);
    }
  };

  const handleSignUp = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError("");

    try {
      const users = getUsers();
      const emailLower = email.toLowerCase();

      if (users[emailLower]) {
        setError("An account with this email already exists");
        setLoading(false);
        return;
      }

      const newUser: User = {
        id: crypto.randomUUID(),
        name,
        email: emailLower,
        createdAt: new Date().toISOString(),
      };

      users[emailLower] = { user: newUser, password };
      saveUsers(users);
      localStorage.setItem(CURRENT_USER_KEY, JSON.stringify(newUser));

      setMode("background");
    } catch (err) {
      setError("Sign up failed. Please try again.");
    } finally {
      setLoading(false);
    }
  };

  const handleBackgroundSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);

    try {
      const users = getUsers();
      const emailLower = email.toLowerCase();

      if (users[emailLower]) {
        users[emailLower].background = background;
        saveUsers(users);
      }

      localStorage.setItem("userBackground", JSON.stringify(background));
      onSuccess();
      onClose();
    } finally {
      setLoading(false);
    }
  };

  const toggleArrayItem = (field: keyof BackgroundData, item: string) => {
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
        <button
          type="button"
          onClick={() => {
            setMode("signup");
            setError("");
          }}
        >
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
          placeholder="Create a password (min 6 characters)"
          minLength={6}
          required
        />
      </div>

      <button type="submit" className={styles.submitBtn} disabled={loading}>
        {loading ? "Creating account..." : "Sign Up"}
      </button>

      <p className={styles.switchMode}>
        Already have an account?{" "}
        <button
          type="button"
          onClick={() => {
            setMode("signin");
            setError("");
          }}
        >
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
              setBackground({
                ...background,
                programmingExperience: e.target.value,
              })
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
            {["Python", "C++", "JavaScript", "Java", "Rust", "Go"].map(
              (lang) => (
                <label key={lang} className={styles.checkbox}>
                  <input
                    type="checkbox"
                    checked={background.programmingLanguages.includes(lang)}
                    onChange={() =>
                      toggleArrayItem("programmingLanguages", lang)
                    }
                  />
                  {lang}
                </label>
              ),
            )}
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
              setBackground({
                ...background,
                roboticsExperience: e.target.value,
              })
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
            {[
              "Arduino",
              "Raspberry Pi",
              "NVIDIA Jetson",
              "ESP32",
              "STM32",
              "ROS Robots",
            ].map((platform) => (
              <label key={platform} className={styles.checkbox}>
                <input
                  type="checkbox"
                  checked={background.hardwarePlatforms.includes(platform)}
                  onChange={() =>
                    toggleArrayItem("hardwarePlatforms", platform)
                  }
                />
                {platform}
              </label>
            ))}
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
