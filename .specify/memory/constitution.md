<!--
SYNC IMPACT REPORT
==================
Version change: 0.0.0 → 1.0.0 (Initial constitution)
Modified principles: N/A (new document)
Added sections:
  - Core Principles (7 principles)
  - Technology Stack Requirements
  - Content Quality Standards
  - Governance
Removed sections: N/A
Templates requiring updates:
  - .specify/templates/plan-template.md ⚠ pending (review Constitution Check section)
  - .specify/templates/spec-template.md ⚠ pending (align with content principles)
  - .specify/templates/tasks-template.md ⚠ pending (align task categories)
Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Spec-Driven Development (NON-NEGOTIABLE)

All features MUST follow the Spec-Driven Development workflow:
- Every chapter, component, or feature starts with a specification (`/sp.specify`)
- Planning MUST precede implementation (`/sp.plan`)
- Tasks MUST be generated before coding (`/sp.tasks`)
- No code changes without corresponding spec documentation
- Prompt History Records (PHR) MUST be created for every significant interaction

**Rationale**: Ensures traceability, maintains consistency across 28 chapters, and enables
AI-assisted development with clear context.

### II. Content-First Architecture

Content quality takes priority over feature complexity:
- Chapter content MUST be technically accurate and pedagogically sound
- Code examples MUST be tested and runnable on specified hardware
- All ROS 2, Gazebo, Isaac Sim examples MUST target Ubuntu 22.04 LTS
- Diagrams and visuals MUST clarify, not decorate
- Each chapter MUST have clear learning objectives and assessments

**Rationale**: The textbook's primary value is educational content; features like chatbot
and podcasts enhance but do not replace quality writing.

### III. Modular Component Design

All interactive features MUST be independently deployable:
- RAG Chatbot: Standalone FastAPI service with clear API contracts
- Podcast Player: Self-contained React component with no external dependencies
- Personalization: Isolated service that does not block core content rendering
- Translation: Client-side or API-based, must not affect page load performance

**Rationale**: Enables incremental delivery, easier debugging, and graceful degradation
if any single feature fails.

### IV. Bilingual Support (English + Urdu)

All user-facing content MUST support bilingual delivery:
- Chapter text: English primary, Urdu translation via button
- Podcasts: Separate audio files for English and Urdu
- UI elements: All buttons and labels MUST have Urdu translations
- Chatbot: MUST understand and respond in both languages

**Rationale**: Serves the Panaversity mission of accessible education for Pakistani
students and Urdu-speaking learners worldwide.

### V. Security & Privacy First

All user data handling MUST follow security best practices:
- Authentication via better-auth.com with secure session management
- User background data stored encrypted in Neon Postgres
- API keys and secrets MUST use environment variables, never hardcoded
- No PII in logs or error messages
- HTTPS required for all production endpoints

**Rationale**: Protecting user data is non-negotiable, especially for educational
platforms serving students.

### VI. Performance & Accessibility

The textbook MUST be fast and accessible:
- Initial page load under 3 seconds on 3G connection
- All images MUST have alt text
- Code blocks MUST be screen-reader compatible
- Audio player MUST have keyboard controls
- Lighthouse accessibility score MUST be 90+

**Rationale**: Educational content must be accessible to users with varying internet
speeds and abilities.

### VII. Simplicity Over Complexity (YAGNI)

Implement only what is specified:
- No speculative features or "nice-to-haves" without spec approval
- Prefer standard Docusaurus components over custom implementations
- Use existing libraries (OpenAI SDK, FastAPI, React) over custom solutions
- Minimize dependencies; each new package MUST be justified
- Avoid premature optimization

**Rationale**: Hackathon timeline demands focus; complexity risks missing deadlines.

## Technology Stack Requirements

The following technology choices are MANDATORY for this project:

| Layer | Technology | Version/Tier | Justification |
|-------|------------|--------------|---------------|
| Book Platform | Docusaurus | 3.x | Required by hackathon |
| Deployment | GitHub Pages | - | Required by hackathon |
| Backend API | FastAPI | Latest | Python ecosystem, async support |
| Database | Neon Serverless Postgres | Free tier | Required by hackathon |
| Vector Store | Qdrant Cloud | Free tier | Required by hackathon |
| AI/LLM | OpenAI Agents SDK | Latest | Required by hackathon |
| Authentication | better-auth.com | - | Bonus requirement |
| Podcast Gen | NotebookLM / OpenAI TTS | - | AI-generated audio |
| Frontend | React + TypeScript | 18.x | Docusaurus default |

**Deviations from this stack require explicit justification and approval.**

## Content Quality Standards

### Chapter Structure (MANDATORY)

Every chapter MUST include:
1. **Learning Objectives**: 3-5 measurable outcomes
2. **Prerequisites**: Links to prior chapters or external knowledge
3. **Core Content**: Theory with practical examples
4. **Code Examples**: Tested, runnable, with expected output
5. **Exercises**: Minimum 3 per chapter with solutions
6. **Summary**: Key takeaways in bullet form
7. **References**: Academic and industry sources

### Code Example Standards

- All Python code MUST be PEP 8 compliant
- ROS 2 code MUST follow REP (ROS Enhancement Proposal) conventions
- Every code block MUST specify the filename and language
- Complex examples MUST include inline comments
- Hardware-specific code MUST note requirements (GPU, Jetson, etc.)

### Media Standards

- Images: WebP format preferred, max 500KB, descriptive filenames
- Diagrams: Created with standard tools (draw.io, Mermaid), source files retained
- Audio: MP3 format, 128kbps minimum, normalized volume levels
- Videos: External links only (YouTube/Vimeo), no self-hosted video

## Governance

### Constitution Authority

This constitution supersedes all other project documentation. In case of conflict:
1. Constitution takes precedence
2. Then approved specs (`specs/*/spec.md`)
3. Then approved plans (`specs/*/plan.md`)
4. Then task definitions (`specs/*/tasks.md`)

### Amendment Process

1. Propose amendment via PHR with stage `constitution`
2. Document rationale and impact analysis
3. Update version according to semantic versioning:
   - MAJOR: Principle removal or fundamental redefinition
   - MINOR: New principle or section added
   - PATCH: Clarification or typo fix
4. Update Sync Impact Report at top of file
5. Propagate changes to dependent templates

### Compliance Verification

All pull requests MUST verify:
- [ ] Changes align with stated principles
- [ ] No hardcoded secrets or API keys
- [ ] Code examples are tested
- [ ] Accessibility standards maintained
- [ ] PHR created for significant work

### Decision Records

Architecturally significant decisions MUST be documented:
- Use `/sp.adr` command for Architecture Decision Records
- ADRs stored in `history/adr/`
- Link ADRs from relevant specs and plans

**Version**: 1.0.0 | **Ratified**: 2026-02-05 | **Last Amended**: 2026-02-05
