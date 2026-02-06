import { createAuthClient } from "better-auth/react";

// Auth server URL - change this for production
const AUTH_SERVER_URL =
  process.env.NODE_ENV === "production"
    ? "https://physical-ai-auth-server.vercel.app" // Production auth server
    : "http://localhost:3001";

export const authClient = createAuthClient({
  baseURL: AUTH_SERVER_URL,
});

export const { signIn, signUp, signOut, useSession } = authClient;

// User background interface
export interface UserBackground {
  programmingExperience: "none" | "beginner" | "intermediate" | "advanced";
  programmingLanguages: string[];
  rosExperience: "none" | "beginner" | "intermediate" | "advanced";
  aiMlExperience: "none" | "beginner" | "intermediate" | "advanced";
  roboticsExperience: "none" | "beginner" | "intermediate" | "advanced";
  hardwarePlatforms: string[];
  hasJetson: boolean;
  hasRobot: boolean;
  learningGoals: string[];
  preferredPace: "slow" | "normal" | "fast";
}

// Helper to update user background
export async function updateUserBackground(
  background: UserBackground,
): Promise<boolean> {
  try {
    const response = await fetch(`${AUTH_SERVER_URL}/api/user/background`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      credentials: "include",
      body: JSON.stringify(background),
    });

    return response.ok;
  } catch (error) {
    console.error("Failed to update background:", error);
    return false;
  }
}

// Helper to get user profile
export async function getUserProfile() {
  try {
    const response = await fetch(`${AUTH_SERVER_URL}/api/user/profile`, {
      method: "GET",
      credentials: "include",
    });

    if (response.ok) {
      return await response.json();
    }
    return null;
  } catch (error) {
    console.error("Failed to get profile:", error);
    return null;
  }
}
