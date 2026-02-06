import { betterAuth } from "better-auth";
import { createClient } from "@libsql/client";

// Turso database client
const tursoClient = createClient({
  url: process.env.TURSO_DATABASE_URL || "",
  authToken: process.env.TURSO_AUTH_TOKEN || "",
});

export const auth = betterAuth({
  database: {
    type: "sqlite",
    client: tursoClient as any,
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
