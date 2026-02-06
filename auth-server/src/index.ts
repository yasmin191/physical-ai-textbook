import express from "express";
import cors from "cors";
import { toNodeHandler } from "better-auth/node";
import { auth } from "./auth.js";
import dotenv from "dotenv";

dotenv.config();

const app = express();
const PORT = process.env.PORT || 3001;

// CORS configuration
app.use(
  cors({
    origin: [
      "http://localhost:3000",
      "http://localhost:3001",
      "https://yasmin191.github.io",
      "https://physical-ai-textbook.vercel.app",
    ],
    credentials: true,
    methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allowedHeaders: ["Content-Type", "Authorization"],
  })
);

// Parse JSON bodies
app.use(express.json());

// Health check endpoint
app.get("/health", (req, res) => {
  res.json({ status: "ok", timestamp: new Date().toISOString() });
});

// Better Auth handler - handles all /api/auth/* routes
app.all("/api/auth/*", toNodeHandler(auth));

// User profile endpoint to get/update background info
app.get("/api/user/profile", async (req, res) => {
  try {
    const session = await auth.api.getSession({
      headers: req.headers as any,
    });

    if (!session) {
      return res.status(401).json({ error: "Unauthorized" });
    }

    res.json({ user: session.user });
  } catch (error) {
    console.error("Profile error:", error);
    res.status(500).json({ error: "Failed to get profile" });
  }
});

// Update user background
app.post("/api/user/background", async (req, res) => {
  try {
    const session = await auth.api.getSession({
      headers: req.headers as any,
    });

    if (!session) {
      return res.status(401).json({ error: "Unauthorized" });
    }

    const {
      programmingExperience,
      programmingLanguages,
      rosExperience,
      aiMlExperience,
      roboticsExperience,
      hardwarePlatforms,
      hasJetson,
      hasRobot,
      learningGoals,
      preferredPace,
    } = req.body;

    // Update user with background info
    await auth.api.updateUser({
      body: {
        programmingExperience,
        programmingLanguages: JSON.stringify(programmingLanguages),
        rosExperience,
        aiMlExperience,
        roboticsExperience,
        hardwarePlatforms: JSON.stringify(hardwarePlatforms),
        hasJetson,
        hasRobot,
        learningGoals: JSON.stringify(learningGoals),
        preferredPace,
      },
      headers: req.headers as any,
    });

    res.json({ success: true });
  } catch (error) {
    console.error("Background update error:", error);
    res.status(500).json({ error: "Failed to update background" });
  }
});

app.listen(PORT, () => {
  console.log(`Auth server running on http://localhost:${PORT}`);
  console.log(`Health check: http://localhost:${PORT}/health`);
});
