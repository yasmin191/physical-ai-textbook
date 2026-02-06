# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `001-physical-ai-textbook` | **Date**: 2026-02-05 | **Spec**: [spec.md](./spec.md)  
**Input**: Feature specification from `/specs/001-physical-ai-textbook/spec.md`

## Summary

Build a comprehensive web-based textbook for the Physical AI & Humanoid Robotics course using Docusaurus, with an integrated RAG chatbot, bilingual podcast player (English/Urdu), content translation, and user personalization features. The textbook covers 28 chapters across 4 modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA) aligned to a 13-week curriculum.

## Technical Context

**Language/Version**: TypeScript 5.x (frontend), Python 3.11 (backend)  
**Primary Dependencies**: Docusaurus 3.x, React 18.x, FastAPI, OpenAI SDK, better-auth  
**Storage**: Neon Serverless Postgres (user data), Qdrant Cloud (vector embeddings)  
**Testing**: Jest (frontend), pytest (backend), Playwright (E2E)  
**Target Platform**: Web (GitHub Pages for static content, serverless for API)  
**Project Type**: Web application (frontend + backend)  
**Performance Goals**: <3s page load, <5s chatbot response, 90+ Lighthouse score  
**Constraints**: Free tier limits on Neon/Qdrant, hackathon deadline Nov 30, 2025  
**Scale/Scope**: 28 chapters, ~1000 concurrent users, bilingual content

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| I. Spec-Driven Development | ✅ PASS | Spec created before plan; PHRs being recorded |
| II. Content-First Architecture | ✅ PASS | 28 chapters prioritized as P1 user story |
| III. Modular Component Design | ✅ PASS | Chatbot, Podcast, Personalization as separate components |
| IV. Bilingual Support | ✅ PASS | English + Urdu for podcasts, translation, chatbot |
| V. Security & Privacy First | ✅ PASS | better-auth for auth, env vars for secrets |
| VI. Performance & Accessibility | ✅ PASS | <3s load time, 90+ Lighthouse in success criteria |
| VII. Simplicity (YAGNI) | ✅ PASS | Using standard Docusaurus, existing libraries |

**Stack Compliance**: All technologies match constitution requirements.

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-textbook/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output (API contracts)
│   ├── chat-api.yaml
│   └── auth-api.yaml
├── checklists/
│   └── requirements.md  # Spec quality checklist
└── tasks.md             # Phase 2 output (via /sp.tasks)
```

### Source Code (repository root)

```text
textbook/                        # Docusaurus frontend
├── docusaurus.config.ts
├── sidebars.ts
├── package.json
├── tsconfig.json
├── docs/                        # Book content (Markdown)
│   ├── intro.md
│   ├── module-1-ros2/
│   │   ├── 01-intro-physical-ai.md
│   │   ├── 02-humanoid-landscape.md
│   │   ├── 03-sensor-systems.md
│   │   ├── 04-ros2-architecture.md
│   │   ├── 05-nodes-topics-services.md
│   │   ├── 06-ros2-python-packages.md
│   │   ├── 07-launch-files.md
│   │   └── 08-urdf.md
│   ├── module-2-simulation/
│   │   ├── 09-gazebo-setup.md
│   │   ├── 10-urdf-sdf.md
│   │   ├── 11-physics-simulation.md
│   │   ├── 12-sensor-simulation.md
│   │   └── 13-unity-hri.md
│   ├── module-3-isaac/
│   │   ├── 14-isaac-overview.md
│   │   ├── 15-isaac-sim.md
│   │   ├── 16-isaac-ros.md
│   │   ├── 17-vslam-navigation.md
│   │   ├── 18-nav2-bipedal.md
│   │   ├── 19-rl-control.md
│   │   └── 20-sim-to-real.md
│   ├── module-4-vla/
│   │   ├── 21-humanoid-kinematics.md
│   │   ├── 22-bipedal-locomotion.md
│   │   ├── 23-manipulation-grasping.md
│   │   ├── 24-voice-to-action.md
│   │   ├── 25-cognitive-planning.md
│   │   ├── 26-conversational-robotics.md
│   │   ├── 27-multimodal-interaction.md
│   │   └── 28-capstone.md
│   └── appendices/
│       ├── a-hardware-setup.md
│       ├── b-ubuntu-ros2-install.md
│       ├── c-isaac-sim-setup.md
│       ├── d-jetson-setup.md
│       └── e-math-foundations.md
├── src/
│   ├── components/
│   │   ├── RAGChatbot/
│   │   │   ├── index.tsx
│   │   │   ├── ChatWindow.tsx
│   │   │   ├── MessageList.tsx
│   │   │   ├── InputArea.tsx
│   │   │   └── styles.module.css
│   │   ├── PodcastPlayer/
│   │   │   ├── index.tsx
│   │   │   ├── AudioControls.tsx
│   │   │   ├── LanguageToggle.tsx
│   │   │   └── styles.module.css
│   │   ├── TranslateButton/
│   │   │   ├── index.tsx
│   │   │   └── styles.module.css
│   │   ├── PersonalizeButton/
│   │   │   ├── index.tsx
│   │   │   └── styles.module.css
│   │   └── AuthProvider/
│   │       ├── index.tsx
│   │       ├── SignInModal.tsx
│   │       ├── SignUpModal.tsx
│   │       └── UserMenu.tsx
│   ├── hooks/
│   │   ├── useAuth.ts
│   │   ├── useChat.ts
│   │   ├── useTranslation.ts
│   │   └── usePersonalization.ts
│   ├── services/
│   │   ├── chatService.ts
│   │   ├── authService.ts
│   │   └── translationService.ts
│   ├── css/
│   │   └── custom.css
│   └── pages/
│       └── index.tsx
├── static/
│   ├── img/
│   │   └── [chapter diagrams]
│   └── audio/
│       ├── en/
│       │   ├── chapter-01.mp3
│       │   └── ... (28 files)
│       └── ur/
│           ├── chapter-01.mp3
│           └── ... (28 files)
└── tests/
    ├── components/
    └── e2e/

api/                             # FastAPI backend
├── main.py
├── requirements.txt
├── .env.example
├── routes/
│   ├── __init__.py
│   ├── chat.py
│   ├── auth.py
│   └── personalization.py
├── services/
│   ├── __init__.py
│   ├── rag_service.py
│   ├── embedding_service.py
│   ├── qdrant_service.py
│   ├── translation_service.py
│   └── personalization_service.py
├── models/
│   ├── __init__.py
│   ├── user.py
│   ├── chat.py
│   └── preference.py
├── scripts/
│   ├── ingest_content.py
│   └── generate_embeddings.py
└── tests/
    ├── test_chat.py
    ├── test_auth.py
    └── test_rag.py
```

**Structure Decision**: Web application architecture with:
- `textbook/` - Docusaurus static site for book content and UI components
- `api/` - FastAPI backend for chatbot, auth, and personalization services

This separation enables:
1. Static hosting on GitHub Pages (free, fast CDN)
2. Serverless API deployment (Vercel/Railway for backend)
3. Independent scaling of content vs. interactive features

## Complexity Tracking

No constitution violations requiring justification. All choices align with mandated stack.

## Architecture Decisions

### ADR-001: Separate Static Site and API

**Decision**: Host Docusaurus on GitHub Pages, API on separate serverless platform.

**Rationale**: 
- GitHub Pages is free and required by hackathon
- GitHub Pages cannot run server-side code
- FastAPI needs Python runtime for RAG/OpenAI integration

**Alternatives Rejected**:
- Full-stack on Vercel: Would require Docusaurus rebuild for Next.js
- Self-hosted: Adds complexity, cost, and maintenance burden

### ADR-002: Vector Storage Strategy

**Decision**: Use Qdrant Cloud free tier with chunked chapter embeddings.

**Rationale**:
- Free tier allows 1GB storage, sufficient for 28 chapters
- Native Python client integrates well with FastAPI
- Better semantic search than keyword-based alternatives

**Alternatives Rejected**:
- Pinecone: More expensive, similar capabilities
- Local FAISS: No persistence, harder to scale

### ADR-003: Podcast Generation Approach

**Decision**: Use NotebookLM for conversational podcast generation, fallback to OpenAI TTS.

**Rationale**:
- NotebookLM produces natural conversational audio
- Free to use with Google account
- OpenAI TTS as backup for programmatic generation

**Alternatives Rejected**:
- Manual recording: Time-prohibitive for 56 audio files (28 × 2 languages)
- ElevenLabs: Excellent quality but costly at scale

## Implementation Phases Overview

| Phase | Focus | Key Deliverables | Duration |
|-------|-------|------------------|----------|
| 1 | Project Setup | Docusaurus init, GitHub repo, CI/CD | Day 1 |
| 2 | Core Content | Module 1-4 chapters (28 total) | Days 2-6 |
| 3 | RAG Chatbot | FastAPI, Qdrant, OpenAI integration | Days 7-8 |
| 4 | Podcast Feature | Audio generation, player component | Days 9-10 |
| 5 | Bonus Features | Auth, personalization, translation | Days 10-11 |
| 6 | Polish & Deploy | Testing, demo video, submission | Days 11-12 |

## Risk Register

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Qdrant free tier limits | Medium | High | Efficient chunking, monitor usage |
| NotebookLM unavailable | Low | Medium | Fallback to OpenAI TTS |
| OpenAI API rate limits | Medium | High | Implement caching, queue requests |
| Time overrun on content | High | Critical | Prioritize Module 1, skeleton others |
| GitHub Pages build fails | Low | Medium | Test deployment early, have backup |

## Dependencies & External Services

| Service | Purpose | Account Required | Free Tier Limits |
|---------|---------|------------------|------------------|
| GitHub | Repo + Pages hosting | Yes | Unlimited public repos |
| Qdrant Cloud | Vector storage | Yes | 1GB storage, 1M vectors |
| Neon Postgres | User database | Yes | 0.5GB storage, 1 project |
| OpenAI | Embeddings + Chat | Yes | Pay-per-use ($5 credit) |
| better-auth | Authentication | No | Open source |
| NotebookLM | Podcast generation | Google account | Free |

## Next Steps

1. Create `research.md` - Document technology research findings
2. Create `data-model.md` - Define database schema
3. Create `contracts/` - API specifications
4. Create `quickstart.md` - Developer setup guide
5. Run `/sp.tasks` - Generate implementation tasks
