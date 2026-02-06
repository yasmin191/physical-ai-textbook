# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

**Date**: 2026-02-05  
**Branch**: `001-physical-ai-textbook`

## Prerequisites

### Required Software

| Software | Version | Purpose |
|----------|---------|---------|
| Node.js | 18.x or 20.x | Docusaurus runtime |
| npm | 9.x+ | Package management |
| Python | 3.11+ | Backend API |
| Git | 2.x+ | Version control |

### Required Accounts

| Service | URL | Purpose |
|---------|-----|---------|
| GitHub | github.com | Repository & Pages hosting |
| OpenAI | platform.openai.com | Embeddings & Chat API |
| Qdrant Cloud | cloud.qdrant.io | Vector database |
| Neon | neon.tech | PostgreSQL database |
| Vercel (optional) | vercel.com | API deployment |

## Project Setup

### 1. Clone Repository

```bash
git clone https://github.com/[username]/hackathon_1.git
cd hackathon_1
git checkout 001-physical-ai-textbook
```

### 2. Setup Frontend (Docusaurus)

```bash
# Navigate to textbook directory
cd textbook

# Install dependencies
npm install

# Start development server
npm run start
```

The textbook will be available at `http://localhost:3000`.

### 3. Setup Backend (FastAPI)

```bash
# Navigate to api directory
cd api

# Create virtual environment
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# macOS/Linux:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Copy environment template
cp .env.example .env

# Edit .env with your credentials
```

### 4. Configure Environment Variables

Create `api/.env` with the following:

```env
# OpenAI
OPENAI_API_KEY=sk-your-api-key-here

# Neon Postgres
DATABASE_URL=postgres://user:password@ep-xxx.region.aws.neon.tech/dbname?sslmode=require

# Qdrant Cloud
QDRANT_URL=https://xxx-xxx.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key

# Authentication
AUTH_SECRET=generate-a-random-32-char-string
JWT_EXPIRY=3600

# CORS
ALLOWED_ORIGINS=http://localhost:3000,https://[username].github.io
```

### 5. Initialize Database

```bash
# Run migrations
cd api
python -m alembic upgrade head

# Or manually run SQL
psql $DATABASE_URL -f migrations/001_create_tables.sql
```

### 6. Index Book Content for RAG

```bash
# Generate embeddings for all chapters
cd api
python scripts/ingest_content.py --source ../textbook/docs
```

This will:
- Read all Markdown files from `textbook/docs/`
- Chunk content by headers
- Generate OpenAI embeddings
- Store in Qdrant collection

### 7. Start Backend Server

```bash
cd api
uvicorn main:app --reload --port 8000
```

API will be available at `http://localhost:8000`.

## Development Workflow

### Writing Book Content

1. Create new chapter in `textbook/docs/module-X/`:

```bash
touch textbook/docs/module-1-ros2/05-nodes-topics-services.md
```

2. Follow chapter template (see `specs/001-physical-ai-textbook/data-model.md`):

```markdown
---
id: chapter-05
title: Nodes, Topics, Services, and Actions
sidebar_label: 5. ROS 2 Communication
sidebar_position: 5
---

import PodcastPlayer from '@site/src/components/PodcastPlayer';

<PodcastPlayer 
  englishSrc="/audio/en/chapter-05.mp3"
  urduSrc="/audio/ur/chapter-05.mp3"
/>

## Learning Objectives

1. Understand ROS 2 communication patterns
2. ...

## Prerequisites

- [Chapter 4: ROS 2 Architecture](./04-ros2-architecture)

## Content

[Your content here]

## Exercises

## Summary

## References
```

3. Update sidebar in `textbook/sidebars.ts` if needed.

### Building Components

React components go in `textbook/src/components/`:

```tsx
// textbook/src/components/MyComponent/index.tsx
import React from 'react';
import styles from './styles.module.css';

interface Props {
  title: string;
}

export default function MyComponent({ title }: Props): JSX.Element {
  return <div className={styles.container}>{title}</div>;
}
```

### Adding API Endpoints

Add routes in `api/routes/`:

```python
# api/routes/my_route.py
from fastapi import APIRouter, HTTPException

router = APIRouter(prefix="/my-endpoint", tags=["MyFeature"])

@router.get("/")
async def get_items():
    return {"items": []}

# Register in api/main.py
from routes.my_route import router as my_router
app.include_router(my_router)
```

## Testing

### Frontend Tests

```bash
cd textbook
npm run test
```

### Backend Tests

```bash
cd api
pytest
```

### End-to-End Tests

```bash
cd textbook
npm run test:e2e
```

## Deployment

### Deploy Frontend to GitHub Pages

```bash
cd textbook

# Build static site
npm run build

# Deploy to GitHub Pages
GIT_USER=<username> npm run deploy
```

### Deploy Backend to Vercel

1. Connect repository to Vercel
2. Set root directory to `api`
3. Configure environment variables in Vercel dashboard
4. Deploy

### Manual Deployment

```bash
# Build frontend
cd textbook
npm run build

# Copy build to docs folder (for GitHub Pages)
cp -r build ../docs

# Commit and push
git add .
git commit -m "Deploy textbook"
git push origin main
```

## Troubleshooting

### Common Issues

**Issue**: `npm install` fails with ERESOLVE

```bash
# Solution: Use legacy peer deps
npm install --legacy-peer-deps
```

**Issue**: Qdrant connection timeout

```bash
# Check Qdrant URL and API key in .env
# Ensure IP is whitelisted in Qdrant Cloud console
```

**Issue**: OpenAI rate limit

```bash
# Implement retry with exponential backoff
# Or upgrade API tier
```

**Issue**: Database migration fails

```bash
# Check DATABASE_URL format
# Ensure Neon project is active (not suspended)
```

### Getting Help

1. Check the [spec.md](./spec.md) for requirements
2. Review [plan.md](./plan.md) for architecture decisions
3. Consult [data-model.md](./data-model.md) for schema details
4. See API contracts in [contracts/](./contracts/)

## File Reference

| Path | Purpose |
|------|---------|
| `textbook/` | Docusaurus frontend |
| `textbook/docs/` | Book chapters (Markdown) |
| `textbook/src/components/` | React components |
| `textbook/static/audio/` | Podcast MP3 files |
| `api/` | FastAPI backend |
| `api/routes/` | API endpoints |
| `api/services/` | Business logic |
| `api/models/` | Data models |
| `specs/001-physical-ai-textbook/` | Project documentation |
