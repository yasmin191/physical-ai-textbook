import React, { useState, useEffect } from "react";
import AuthModal from "../AuthModal";
import styles from "./styles.module.css";

interface User {
  id: string;
  name: string;
  email: string;
}

export default function AuthButton() {
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [user, setUser] = useState<User | null>(null);
  const [showDropdown, setShowDropdown] = useState(false);

  useEffect(() => {
    // Check for stored user on mount
    const storedUser = localStorage.getItem("user");
    if (storedUser) {
      try {
        setUser(JSON.parse(storedUser));
      } catch (e) {
        localStorage.removeItem("user");
      }
    }
  }, []);

  const handleSignOut = () => {
    localStorage.removeItem("user");
    localStorage.removeItem("userBackground");
    setUser(null);
    setShowDropdown(false);
  };

  const handleAuthSuccess = () => {
    const storedUser = localStorage.getItem("user");
    if (storedUser) {
      try {
        setUser(JSON.parse(storedUser));
      } catch (e) {
        // ignore
      }
    }
  };

  if (user) {
    return (
      <div className={styles.userContainer}>
        <button
          className={styles.userButton}
          onClick={() => setShowDropdown(!showDropdown)}
        >
          <span className={styles.avatar}>
            {user.name?.charAt(0).toUpperCase() || "U"}
          </span>
          <span className={styles.userName}>{user.name}</span>
        </button>

        {showDropdown && (
          <div className={styles.dropdown}>
            <div className={styles.dropdownHeader}>
              <strong>{user.name}</strong>
              <span>{user.email}</span>
            </div>
            <hr className={styles.divider} />
            <button
              className={styles.dropdownItem}
              onClick={() => {
                setIsModalOpen(true);
                setShowDropdown(false);
              }}
            >
              Update Background
            </button>
            <button className={styles.dropdownItem} onClick={handleSignOut}>
              Sign Out
            </button>
          </div>
        )}
      </div>
    );
  }

  return (
    <>
      <button className={styles.signInButton} onClick={() => setIsModalOpen(true)}>
        Sign In
      </button>
      <AuthModal
        isOpen={isModalOpen}
        onClose={() => setIsModalOpen(false)}
        onSuccess={handleAuthSuccess}
      />
    </>
  );
}
