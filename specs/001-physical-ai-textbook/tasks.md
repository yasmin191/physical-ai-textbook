# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/001-physical-ai-textbook/`  
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/  
**Branch**: `001-physical-ai-textbook`  
**Created**: 2026-02-05

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1-US6)
- All paths relative to repository root

## Path Conventions

- **Frontend**: `textbook/` (Docusaurus)
- **Backend**: `api/` (FastAPI)
- **Docs content**: `textbook/docs/`
- **Components**: `textbook/src/components/`
- **Audio**: `textbook/static/audio/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization, repository setup, and base configuration

- [ ] T001 Initialize Docusaurus 3.x project in textbook/ directory
- [ ] T002 [P] Configure docusaurus.config.ts with project metadata and i18n settings
- [ ] T003 [P] Create sidebars.ts with 4-module structure (28 chapters + appendices)
- [ ] T004 [P] Configure custom.css with RTL support and accessibility styles in textbook/src/css/custom.css
- [ ] T005 [P] Initialize FastAPI project in api/ with requirements.txt
- [ ] T006 [P] Create api/.env.example with all required environment variables
- [ ] T007 [P] Configure CORS middleware in api/main.py for cross-origin requests
- [ ] T008 Setup GitHub repository with .gitignore for Node and Python
- [ ] T009 [P] Create GitHub Actions workflow for Docusaurus deployment in .github/workflows/deploy.yml
- [ ] T010 [P] Create README.md with project overview and setup instructions

**Checkpoint**: Project structure ready, can start development

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can begin

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T011 Create database migration script in api/migrations/001_create_tables.sql (users, preferences, sessions, messages tables)
- [ ] T012 [P] Configure Neon Postgres connection in api/services/database.py
- [ ] T013 [P] Configure Qdrant Cloud connection in api/services/qdrant_service.py
- [ ] T014 [P] Configure OpenAI client in api/services/openai_service.py
- [ ] T015 Create base Pydantic models in api/models/__init__.py
- [ ] T016 [P] Create User model in api/models/user.py
- [ ] T017 [P] Create ChatMessage model in api/models/chat.py
- [ ] T018 [P] Create UserPreference model in api/models/preference.py
- [ ] T019 Setup API router structure in api/routes/__init__.py
- [ ] T020 Create error handling middleware in api/middleware/error_handler.py
- [ ] T021 Create chapter template MDX component structure in textbook/src/components/ChapterTemplate/index.tsx

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Read Educational Content (Priority: P1) 🎯 MVP

**Goal**: Students can access and read all 28 chapters with proper formatting, code examples, exercises, and navigation

**Independent Test**: Navigate through all chapters, verify content renders correctly, code blocks have syntax highlighting, sidebar navigation works

### Module 1: ROS 2 Fundamentals (Chapters 1-8)

- [ ] T022 [P] [US1] Write Chapter 1: Introduction to Physical AI in textbook/docs/module-1-ros2/01-intro-physical-ai.md
- [ ] T023 [P] [US1] Write Chapter 2: The Humanoid Robotics Landscape in textbook/docs/module-1-ros2/02-humanoid-landscape.md
- [ ] T024 [P] [US1] Write Chapter 3: Sensor Systems in textbook/docs/module-1-ros2/03-sensor-systems.md
- [ ] T025 [P] [US1] Write Chapter 4: ROS 2 Architecture in textbook/docs/module-1-ros2/04-ros2-architecture.md
- [ ] T026 [P] [US1] Write Chapter 5: Nodes, Topics, Services in textbook/docs/module-1-ros2/05-nodes-topics-services.md
- [ ] T027 [P] [US1] Write Chapter 6: ROS 2 Python Packages in textbook/docs/module-1-ros2/06-ros2-python-packages.md
- [ ] T028 [P] [US1] Write Chapter 7: Launch Files in textbook/docs/module-1-ros2/07-launch-files.md
- [ ] T029 [P] [US1] Write Chapter 8: URDF in textbook/docs/module-1-ros2/08-urdf.md

### Module 2: Simulation (Chapters 9-13)

- [ ] T030 [P] [US1] Write Chapter 9: Gazebo Setup in textbook/docs/module-2-simulation/09-gazebo-setup.md
- [ ] T031 [P] [US1] Write Chapter 10: URDF and SDF in textbook/docs/module-2-simulation/10-urdf-sdf.md
- [ ] T032 [P] [US1] Write Chapter 11: Physics Simulation in textbook/docs/module-2-simulation/11-physics-simulation.md
- [ ] T033 [P] [US1] Write Chapter 12: Sensor Simulation in textbook/docs/module-2-simulation/12-sensor-simulation.md
- [ ] T034 [P] [US1] Write Chapter 13: Unity for HRI in textbook/docs/module-2-simulation/13-unity-hri.md

### Module 3: NVIDIA Isaac (Chapters 14-20)

- [ ] T035 [P] [US1] Write Chapter 14: Isaac Platform Overview in textbook/docs/module-3-isaac/14-isaac-overview.md
- [ ] T036 [P] [US1] Write Chapter 15: Isaac Sim in textbook/docs/module-3-isaac/15-isaac-sim.md
- [ ] T037 [P] [US1] Write Chapter 16: Isaac ROS in textbook/docs/module-3-isaac/16-isaac-ros.md
- [ ] T038 [P] [US1] Write Chapter 17: VSLAM Navigation in textbook/docs/module-3-isaac/17-vslam-navigation.md
- [ ] T039 [P] [US1] Write Chapter 18: Nav2 Bipedal in textbook/docs/module-3-isaac/18-nav2-bipedal.md
- [ ] T040 [P] [US1] Write Chapter 19: RL Control in textbook/docs/module-3-isaac/19-rl-control.md
- [ ] T041 [P] [US1] Write Chapter 20: Sim-to-Real in textbook/docs/module-3-isaac/20-sim-to-real.md

### Module 4: Vision-Language-Action (Chapters 21-28)

- [ ] T042 [P] [US1] Write Chapter 21: Humanoid Kinematics in textbook/docs/module-4-vla/21-humanoid-kinematics.md
- [ ] T043 [P] [US1] Write Chapter 22: Bipedal Locomotion in textbook/docs/module-4-vla/22-bipedal-locomotion.md
- [ ] T044 [P] [US1] Write Chapter 23: Manipulation Grasping in textbook/docs/module-4-vla/23-manipulation-grasping.md
- [ ] T045 [P] [US1] Write Chapter 24: Voice to Action in textbook/docs/module-4-vla/24-voice-to-action.md
- [ ] T046 [P] [US1] Write Chapter 25: Cognitive Planning in textbook/docs/module-4-vla/25-cognitive-planning.md
- [ ] T047 [P] [US1] Write Chapter 26: Conversational Robotics in textbook/docs/module-4-vla/26-conversational-robotics.md
- [ ] T048 [P] [US1] Write Chapter 27: Multimodal Interaction in textbook/docs/module-4-vla/27-multimodal-interaction.md
- [ ] T049 [P] [US1] Write Chapter 28: Capstone Project in textbook/docs/module-4-vla/28-capstone.md

### Appendices

- [ ] T050 [P] [US1] Write Appendix A: Hardware Setup Guide in textbook/docs/appendices/a-hardware-setup.md
- [ ] T051 [P] [US1] Write Appendix B: Ubuntu ROS 2 Installation in textbook/docs/appendices/b-ubuntu-ros2-install.md
- [ ] T052 [P] [US1] Write Appendix C: Isaac Sim Setup in textbook/docs/appendices/c-isaac-sim-setup.md
- [ ] T053 [P] [US1] Write Appendix D: Jetson Setup in textbook/docs/appendices/d-jetson-setup.md
- [ ] T054 [P] [US1] Write Appendix E: Math Foundations in textbook/docs/appendices/e-math-foundations.md

### Content Infrastructure

- [ ] T055 [US1] Create intro.md homepage with course overview in textbook/docs/intro.md
- [ ] T056 [US1] Add chapter diagrams and images to textbook/static/img/
- [ ] T057 [US1] Verify all code examples compile on Ubuntu 22.04

**Checkpoint**: User Story 1 complete - Book is fully readable with all 28 chapters

---

## Phase 4: User Story 2 - RAG Chatbot (Priority: P2)

**Goal**: Students can ask questions about textbook content and receive accurate, sourced answers

**Independent Test**: Ask "What is ROS 2?" and verify response cites Chapter 4; test selected-text Q&A on any paragraph

### Backend Implementation

- [ ] T058 [P] [US2] Create embedding service in api/services/embedding_service.py
- [ ] T059 [P] [US2] Create RAG service in api/services/rag_service.py
- [ ] T060 [US2] Create content ingestion script in api/scripts/ingest_content.py
- [ ] T061 [US2] Create chat route with /chat endpoint in api/routes/chat.py
- [ ] T062 [US2] Create session management routes in api/routes/chat.py (create, get, delete session)
- [ ] T063 [US2] Implement language detection for bilingual responses in api/services/rag_service.py
- [ ] T064 [US2] Run content ingestion to populate Qdrant with chapter embeddings

### Frontend Implementation

- [ ] T065 [P] [US2] Create ChatWindow component in textbook/src/components/RAGChatbot/ChatWindow.tsx
- [ ] T066 [P] [US2] Create MessageList component in textbook/src/components/RAGChatbot/MessageList.tsx
- [ ] T067 [P] [US2] Create InputArea component in textbook/src/components/RAGChatbot/InputArea.tsx
- [ ] T068 [US2] Create main RAGChatbot component in textbook/src/components/RAGChatbot/index.tsx
- [ ] T069 [US2] Create chatbot styles in textbook/src/components/RAGChatbot/styles.module.css
- [ ] T070 [US2] Create useChat hook in textbook/src/hooks/useChat.ts
- [ ] T071 [US2] Create chatService in textbook/src/services/chatService.ts
- [ ] T072 [US2] Implement selected-text Q&A feature (highlight → ask) in RAGChatbot component
- [ ] T073 [US2] Integrate RAGChatbot into Docusaurus layout (floating button on all pages)

**Checkpoint**: User Story 2 complete - Chatbot answers questions with source citations

---

## Phase 5: User Story 3 - Chapter Podcasts (Priority: P3)

**Goal**: Students can listen to audio versions of each chapter in English or Urdu

**Independent Test**: Play Chapter 1 podcast in English, switch to Urdu, verify controls work

### Audio Generation (Manual/External)

- [ ] T074 [P] [US3] Generate English podcasts for Module 1 chapters (1-8) using NotebookLM
- [ ] T075 [P] [US3] Generate English podcasts for Module 2 chapters (9-13) using NotebookLM
- [ ] T076 [P] [US3] Generate English podcasts for Module 3 chapters (14-20) using NotebookLM
- [ ] T077 [P] [US3] Generate English podcasts for Module 4 chapters (21-28) using NotebookLM
- [ ] T078 [P] [US3] Generate Urdu podcasts for all chapters using OpenAI TTS
- [ ] T079 [US3] Organize audio files in textbook/static/audio/en/ and textbook/static/audio/ur/

### Frontend Implementation

- [ ] T080 [P] [US3] Create AudioControls component in textbook/src/components/PodcastPlayer/AudioControls.tsx
- [ ] T081 [P] [US3] Create LanguageToggle component in textbook/src/components/PodcastPlayer/LanguageToggle.tsx
- [ ] T082 [US3] Create main PodcastPlayer component in textbook/src/components/PodcastPlayer/index.tsx
- [ ] T083 [US3] Create podcast player styles in textbook/src/components/PodcastPlayer/styles.module.css
- [ ] T084 [US3] Add playback speed control (0.5x, 1x, 1.5x, 2x) to AudioControls
- [ ] T085 [US3] Add download button to PodcastPlayer
- [ ] T086 [US3] Embed PodcastPlayer in all chapter MDX files via import

**Checkpoint**: User Story 3 complete - All chapters have working bilingual podcasts

---

## Phase 6: User Story 4 - Urdu Translation (Priority: P4)

**Goal**: Students can translate chapter content to Urdu with proper RTL formatting

**Independent Test**: Click "Translate to Urdu" on Chapter 1, verify RTL display, toggle back to English

### Backend Implementation

- [ ] T087 [US4] Create translation service in api/services/translation_service.py
- [ ] T088 [US4] Create /translate endpoint in api/routes/translation.py
- [ ] T089 [US4] Implement caching for translations in translation service

### Frontend Implementation

- [ ] T090 [P] [US4] Create TranslateButton component in textbook/src/components/TranslateButton/index.tsx
- [ ] T091 [P] [US4] Create translation styles with RTL support in textbook/src/components/TranslateButton/styles.module.css
- [ ] T092 [US4] Create useTranslation hook in textbook/src/hooks/useTranslation.ts
- [ ] T093 [US4] Create translationService in textbook/src/services/translationService.ts
- [ ] T094 [US4] Implement localStorage caching for translated content
- [ ] T095 [US4] Ensure code blocks remain in English when translating
- [ ] T096 [US4] Embed TranslateButton in all chapter MDX files

**Checkpoint**: User Story 4 complete - Translation works with proper RTL formatting

---

## Phase 7: User Story 6 - Authentication (Priority: P6 - needed for US5)

**Goal**: Users can create accounts, sign in, and maintain sessions

**Independent Test**: Sign up with email, sign out, sign back in, verify session persists

### Backend Implementation

- [ ] T097 [US6] Configure better-auth in api/services/auth_service.py
- [ ] T098 [US6] Create auth routes in api/routes/auth.py (signup, signin, signout, refresh)
- [ ] T099 [US6] Implement password reset flow in api/routes/auth.py
- [ ] T100 [US6] Create background questionnaire validation in signup endpoint

### Frontend Implementation

- [ ] T101 [P] [US6] Create SignInModal component in textbook/src/components/AuthProvider/SignInModal.tsx
- [ ] T102 [P] [US6] Create SignUpModal with background questionnaire in textbook/src/components/AuthProvider/SignUpModal.tsx
- [ ] T103 [P] [US6] Create UserMenu component in textbook/src/components/AuthProvider/UserMenu.tsx
- [ ] T104 [US6] Create main AuthProvider component in textbook/src/components/AuthProvider/index.tsx
- [ ] T105 [US6] Create useAuth hook in textbook/src/hooks/useAuth.ts
- [ ] T106 [US6] Create authService in textbook/src/services/authService.ts
- [ ] T107 [US6] Integrate AuthProvider into Docusaurus layout (navbar)

**Checkpoint**: User Story 6 complete - Users can authenticate and maintain sessions

---

## Phase 8: User Story 5 - Content Personalization (Priority: P5)

**Goal**: Authenticated users can personalize content depth based on their background

**Independent Test**: Create two accounts with different backgrounds, verify same chapter shows different content depth

### Backend Implementation

- [ ] T108 [US5] Create personalization service in api/services/personalization_service.py
- [ ] T109 [US5] Create preferences routes in api/routes/personalization.py
- [ ] T110 [US5] Implement content depth adaptation logic based on user profile

### Frontend Implementation

- [ ] T111 [P] [US5] Create PersonalizeButton component in textbook/src/components/PersonalizeButton/index.tsx
- [ ] T112 [P] [US5] Create personalization styles in textbook/src/components/PersonalizeButton/styles.module.css
- [ ] T113 [US5] Create usePersonalization hook in textbook/src/hooks/usePersonalization.ts
- [ ] T114 [US5] Create PersonalizedContent MDX component for conditional rendering in textbook/src/components/PersonalizedContent/index.tsx
- [ ] T115 [US5] Add beginner/advanced content variants to 5 key chapters
- [ ] T116 [US5] Embed PersonalizeButton in all chapter MDX files

**Checkpoint**: User Story 5 complete - Content adapts to user background

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements, testing, and deployment preparation

- [ ] T117 [P] Run Lighthouse accessibility audit and fix issues to achieve 90+ score
- [ ] T118 [P] Test all features in Chrome, Firefox, Safari, Edge browsers
- [ ] T119 [P] Verify all code examples in chapters compile on Ubuntu 22.04
- [ ] T120 Optimize images (convert to WebP, compress under 500KB)
- [ ] T121 [P] Add meta tags and SEO optimization to docusaurus.config.ts
- [ ] T122 [P] Create error boundary components for graceful failures
- [ ] T123 Deploy API to Vercel and configure environment variables
- [ ] T124 Deploy textbook to GitHub Pages
- [ ] T125 Verify cross-origin API calls work in production
- [ ] T126 Record demo video under 90 seconds showcasing all features
- [ ] T127 Submit project via Google Form with repo, demo link, and video

**Checkpoint**: Project complete and submitted

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup) → Phase 2 (Foundational) → Phases 3-8 (User Stories) → Phase 9 (Polish)
                                              ↓
                                    Can proceed in parallel or sequential
```

### User Story Dependencies

| Story | Can Start After | Dependencies on Other Stories |
|-------|-----------------|-------------------------------|
| US1 (Content) | Phase 2 | None - fully independent |
| US2 (Chatbot) | Phase 2 + some US1 content | Needs content to index |
| US3 (Podcasts) | Phase 2 + US1 content | Needs chapters to generate audio |
| US4 (Translation) | Phase 2 | None - works on any content |
| US5 (Personalization) | Phase 2 + US6 | Requires authentication |
| US6 (Authentication) | Phase 2 | None - fully independent |

### Recommended Execution Order

1. **Setup + Foundational** (T001-T021)
2. **US1 Content** (T022-T057) - MVP milestone
3. **US2 Chatbot** (T058-T073) - Core feature
4. **US3 Podcasts** (T074-T086) - Parallel with chatbot
5. **US6 Auth** (T097-T107) - Needed for personalization
6. **US4 Translation** (T087-T096) - Can be parallel
7. **US5 Personalization** (T108-T116) - After auth
8. **Polish** (T117-T127)

---

## Parallel Execution Examples

### Phase 1 Setup (All Parallel)

```bash
# Can run simultaneously:
Task: T002 Configure docusaurus.config.ts
Task: T003 Create sidebars.ts
Task: T004 Configure custom.css
Task: T005 Initialize FastAPI project
Task: T006 Create .env.example
```

### User Story 1 Chapters (All Parallel)

```bash
# All 28 chapters can be written simultaneously:
Task: T022 Write Chapter 1
Task: T023 Write Chapter 2
...
Task: T049 Write Chapter 28
```

### User Story 2 Components (Parallel)

```bash
# UI components can be built in parallel:
Task: T065 Create ChatWindow component
Task: T066 Create MessageList component
Task: T067 Create InputArea component
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T010)
2. Complete Phase 2: Foundational (T011-T021)
3. Complete Phase 3: US1 - All 28 chapters (T022-T057)
4. **STOP and VALIDATE**: Deploy to GitHub Pages, verify book is readable
5. This alone delivers 100 points (core book requirement)

### Incremental Delivery

| Increment | Stories | Points | Cumulative |
|-----------|---------|--------|------------|
| MVP | US1 (Book) | 100 | 100 |
| +Chatbot | US2 | (included in core) | 100 |
| +Podcasts | US3 | (user-requested) | 100 |
| +Translation | US4 | +50 bonus | 150 |
| +Auth | US6 | (enables US5) | 150 |
| +Personalization | US5 | +50 bonus | 200 |
| +Subagents | (bonus) | +50 bonus | 250 |

### Time Estimates by Phase

| Phase | Tasks | Estimated Days |
|-------|-------|----------------|
| Setup | 10 | 0.5 |
| Foundational | 11 | 0.5 |
| US1 Content | 36 | 4-5 |
| US2 Chatbot | 16 | 1-2 |
| US3 Podcasts | 13 | 1-2 |
| US4 Translation | 10 | 1 |
| US6 Auth | 11 | 1 |
| US5 Personalization | 9 | 1 |
| Polish | 11 | 1 |
| **Total** | **127** | **11-14** |

---

## Task Summary

| Category | Count |
|----------|-------|
| Setup Tasks | 10 |
| Foundational Tasks | 11 |
| US1 (Content) Tasks | 36 |
| US2 (Chatbot) Tasks | 16 |
| US3 (Podcasts) Tasks | 13 |
| US4 (Translation) Tasks | 10 |
| US5 (Personalization) Tasks | 9 |
| US6 (Authentication) Tasks | 11 |
| Polish Tasks | 11 |
| **Total Tasks** | **127** |

### Parallelizable Tasks

- **Setup Phase**: 8 of 10 tasks (80%)
- **Foundational Phase**: 8 of 11 tasks (73%)
- **US1**: 33 of 36 tasks (92%) - all chapters parallel
- **US2**: 4 of 16 tasks (25%)
- **US3**: 6 of 13 tasks (46%)
- **US4**: 2 of 10 tasks (20%)
- **US5**: 2 of 9 tasks (22%)
- **US6**: 3 of 11 tasks (27%)
- **Polish**: 5 of 11 tasks (45%)

---

## Notes

- [P] tasks can run in parallel (different files, no dependencies)
- [US#] label maps task to specific user story
- All chapter writing tasks (T022-T054) can run in parallel
- Commit after each task or logical group
- Test incrementally - don't wait until end
- US1 alone is a complete MVP worth 100 points
