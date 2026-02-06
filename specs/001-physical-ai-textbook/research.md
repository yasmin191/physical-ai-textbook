# Research: Physical AI & Humanoid Robotics Textbook

**Date**: 2026-02-05  
**Branch**: `001-physical-ai-textbook`

## Technology Research

### 1. Docusaurus 3.x Configuration

**Decision**: Use Docusaurus 3.x with TypeScript, MDX support, and custom theme.

**Rationale**:
- Native MDX support allows React components in Markdown
- Built-in search, versioning, and i18n capabilities
- Active community and excellent documentation
- TypeScript provides better developer experience

**Key Configuration**:
```javascript
// docusaurus.config.ts
{
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From Digital Brain to Physical Body',
  url: 'https://[username].github.io',
  baseUrl: '/hackathon_1/',
  onBrokenLinks: 'throw',
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
  },
  presets: [['classic', { docs: { sidebarPath: './sidebars.ts' } }]],
}
```

**Alternatives Considered**:
- GitBook: Less customizable, limited React integration
- MkDocs: Python-based, less suitable for React components
- Next.js MDX: More complex setup, overkill for static content

---

### 2. RAG Chatbot Architecture

**Decision**: FastAPI backend with OpenAI embeddings and Qdrant vector search.

**Rationale**:
- OpenAI `text-embedding-3-small` provides excellent semantic understanding
- Qdrant offers generous free tier (1GB, 1M vectors)
- FastAPI async support handles concurrent chat requests efficiently

**Architecture**:
```
User Query → FastAPI → OpenAI Embedding → Qdrant Search → 
  → Top 5 Chunks → OpenAI GPT-4 → Formatted Response
```

**Chunking Strategy**:
- Split chapters by headers (##, ###)
- Maximum chunk size: 1000 tokens
- Overlap: 100 tokens between chunks
- Estimated chunks: ~500 for 28 chapters

**Alternatives Considered**:
- LangChain: Adds complexity, direct OpenAI SDK simpler
- Pinecone: More expensive, Qdrant free tier sufficient
- ChromaDB: Local-only, no cloud persistence

---

### 3. Podcast Generation

**Decision**: NotebookLM for English, OpenAI TTS for Urdu.

**Rationale**:
- NotebookLM produces natural two-host conversational podcasts
- Free with Google account, no API limits
- OpenAI TTS supports Urdu with `alloy` voice
- Manual upload to static/audio/ directory

**Workflow**:
1. Export chapter Markdown to plain text
2. Upload to NotebookLM, generate English podcast (~10 min each)
3. Download MP3, rename to `chapter-XX.mp3`
4. For Urdu: Translate text, use OpenAI TTS API
5. Store in `static/audio/en/` and `static/audio/ur/`

**File Naming Convention**:
```
static/audio/
├── en/
│   ├── chapter-01.mp3  # Intro to Physical AI
│   ├── chapter-02.mp3  # Humanoid Landscape
│   └── ...
└── ur/
    ├── chapter-01.mp3
    └── ...
```

**Alternatives Considered**:
- ElevenLabs: High quality but expensive ($22/month for 100K chars)
- Manual recording: Time-prohibitive (56 recordings)
- Amazon Polly: Limited Urdu support

---

### 4. Authentication with better-auth

**Decision**: Use better-auth.com with email/password provider.

**Rationale**:
- Open source, no vendor lock-in
- Simple integration with React and FastAPI
- Supports session management and password reset
- No monthly fees (unlike Auth0, Clerk)

**Implementation**:
```typescript
// Frontend: src/services/authService.ts
import { createAuthClient } from 'better-auth/client';

export const authClient = createAuthClient({
  baseUrl: process.env.API_URL,
});
```

```python
# Backend: api/routes/auth.py
from better_auth import BetterAuth

auth = BetterAuth(
    database_url=os.getenv("DATABASE_URL"),
    secret=os.getenv("AUTH_SECRET"),
)
```

**Alternatives Considered**:
- Auth0: Free tier limited, complex setup
- Supabase Auth: Would require Supabase for DB too
- Custom JWT: Security risks, more code to maintain

---

### 5. Translation Strategy

**Decision**: Client-side translation with OpenAI GPT-4 API.

**Rationale**:
- Real-time translation without pre-generating all content
- GPT-4 produces high-quality Urdu translations
- Cache translations in localStorage for repeat visits
- Fallback to showing original if API fails

**Implementation**:
```typescript
// Frontend: src/hooks/useTranslation.ts
async function translateToUrdu(content: string): Promise<string> {
  const cached = localStorage.getItem(`urdu:${hash(content)}`);
  if (cached) return cached;
  
  const response = await fetch('/api/translate', {
    method: 'POST',
    body: JSON.stringify({ content, targetLang: 'ur' }),
  });
  const { translation } = await response.json();
  localStorage.setItem(`urdu:${hash(content)}`, translation);
  return translation;
}
```

**Alternatives Considered**:
- Google Translate API: Less natural for technical content
- Pre-translated static files: 56 extra files to maintain
- DeepL: No Urdu support

---

### 6. Personalization Approach

**Decision**: Rule-based content variants based on user profile.

**Rationale**:
- Simple to implement within hackathon timeline
- Three experience levels: beginner, intermediate, advanced
- Content variants embedded in MDX with conditional rendering
- No AI generation needed (too slow, costly)

**Implementation**:
```mdx
// In chapter MDX file
import { PersonalizedContent } from '@site/src/components/PersonalizedContent';

<PersonalizedContent
  beginner={<>
    ## What is ROS 2?
    ROS 2 (Robot Operating System 2) is a set of software libraries...
    [Detailed explanation with analogies]
  </>}
  advanced={<>
    ## ROS 2 Overview
    DDS-based middleware with QoS policies...
    [Technical deep-dive, assumes familiarity]
  </>}
/>
```

**Alternatives Considered**:
- AI-generated personalization: Too slow, expensive, unpredictable
- Multiple chapter versions: Too much content to write
- No personalization: Loses bonus points

---

### 7. Deployment Architecture

**Decision**: GitHub Pages (static) + Vercel (API).

**Rationale**:
- GitHub Pages: Free, fast CDN, required by hackathon
- Vercel: Free tier supports Python serverless functions
- Automatic deployments from Git pushes
- CORS configured for cross-origin API calls

**Deployment Flow**:
```
GitHub Push → 
  ├─→ GitHub Actions → Build Docusaurus → Deploy to Pages
  └─→ Vercel → Deploy FastAPI → Serverless Functions
```

**Environment Variables**:
```
# Vercel (api/)
OPENAI_API_KEY=sk-...
DATABASE_URL=postgres://...@neon.tech/...
QDRANT_URL=https://...qdrant.io
QDRANT_API_KEY=...
AUTH_SECRET=...

# GitHub Pages (textbook/)
NEXT_PUBLIC_API_URL=https://api.vercel.app
```

**Alternatives Considered**:
- Railway: Good but adds cost after free tier
- Render: Slower cold starts
- AWS Lambda: More complex setup

---

## Content Research

### ROS 2 Best Practices (Module 1)

**Sources Reviewed**:
- ROS 2 Documentation (docs.ros.org)
- "Programming Robots with ROS 2" by Macenski et al.
- ROS 2 Humble tutorials

**Key Decisions**:
- Target ROS 2 Humble (LTS until 2027)
- Use Python (rclpy) for all examples (more accessible than C++)
- Include both simulation and real-robot considerations
- Reference Turtlebot3 as example platform

---

### NVIDIA Isaac Best Practices (Module 3)

**Sources Reviewed**:
- NVIDIA Isaac Sim Documentation
- Isaac ROS GitHub repositories
- GTC 2024/2025 presentations

**Key Decisions**:
- Target Isaac Sim 4.x (latest stable)
- Focus on Isaac ROS packages for perception
- Include synthetic data generation examples
- Cover Nav2 integration for navigation

---

### VLA Models (Module 4)

**Sources Reviewed**:
- GR00T N1 paper (NVIDIA, March 2025)
- Helix by Figure AI (February 2025)
- OpenAI Whisper documentation

**Key Decisions**:
- Cover VLA concept theoretically (no training in scope)
- Use Whisper for voice-to-text examples
- Integrate GPT-4 for cognitive planning demos
- Capstone combines all modules conceptually

---

## Resolved Clarifications

All technical context items resolved. No NEEDS CLARIFICATION markers remain.

| Item | Resolution | Source |
|------|------------|--------|
| Frontend framework | Docusaurus 3.x with React 18 | Hackathon requirement |
| Backend framework | FastAPI (Python 3.11) | Constitution mandate |
| Vector database | Qdrant Cloud free tier | Cost analysis |
| Auth provider | better-auth.com | Hackathon bonus requirement |
| Podcast generation | NotebookLM + OpenAI TTS | Feasibility analysis |
| Translation method | OpenAI GPT-4 API | Quality comparison |
| Personalization | Rule-based MDX variants | Timeline constraints |
