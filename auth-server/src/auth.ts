import { betterAuth } from "better-auth";
import { createClient } from "@libsql/client";

// Use Turso/LibSQL for serverless compatibility
// For hackathon demo, using in-memory SQLite
const client = createClient({
  url: process.env.DATABASE_URL || "file:local.db",
});

export const auth = betterAuth({
  database: {
    provider: "sqlite",
    url: process.env.DATABASE_URL || "file:local.db",
  },
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
  },
  user: {
    additionalFields: {
      programmingExperience: {
        type: "string",
        required: false,
      },
      programmingLanguages: {
        type: "string",
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
      roboticsExperience: {
        type: "string",
        required: false,
      },
      hardwarePlatforms: {
        type: "string",
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
      learningGoals: {
        type: "string",
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
