import { betterAuth } from "better-auth";
import Database from "better-sqlite3";
import path from "path";
import { fileURLToPath } from "url";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const dbPath = path.join(__dirname, "..", "data", "auth.db");

export const auth = betterAuth({
  database: new Database(dbPath),
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
  },
  user: {
    additionalFields: {
      // Software background
      programmingExperience: {
        type: "string",
        required: false,
      },
      programmingLanguages: {
        type: "string", // JSON array stored as string
        required: false,
      },
      rosExperience: {
        type: "string",
        required: false,
      },
      aiMlExperience: {
        type: "string",
        required: false,
      },
      // Hardware background
      roboticsExperience: {
        type: "string",
        required: false,
      },
      hardwarePlatforms: {
        type: "string", // JSON array stored as string
        required: false,
      },
      hasJetson: {
        type: "boolean",
        required: false,
      },
      hasRobot: {
        type: "boolean",
        required: false,
      },
      // Learning goals
      learningGoals: {
        type: "string", // JSON array stored as string
        required: false,
      },
      preferredPace: {
        type: "string",
        required: false,
      },
    },
  },
  trustedOrigins: [
    "http://localhost:3000",
    "http://localhost:3001",
    "https://yasmin191.github.io",
    "https://physical-ai-textbook.vercel.app",
  ],
});

export type Auth = typeof auth;
