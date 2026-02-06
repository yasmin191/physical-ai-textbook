# Data Model: Physical AI & Humanoid Robotics Textbook

**Date**: 2026-02-05  
**Branch**: `001-physical-ai-textbook`

## Overview

This document defines the data structures for the textbook platform, covering:
1. **Neon Postgres**: User accounts, preferences, chat history
2. **Qdrant Cloud**: Chapter embeddings for RAG search
3. **Static Files**: Markdown content, audio files, images

## Entity Relationship Diagram

```
┌─────────────┐     ┌──────────────────┐     ┌─────────────────┐
│    User     │────<│  UserPreference  │     │   ChatSession   │
└─────────────┘     └──────────────────┘     └─────────────────┘
       │                                            │
       │                                            │
       └────────────────────┬───────────────────────┘
                            │
                     ┌──────┴──────┐
                     │ ChatMessage │
                     └─────────────┘

┌─────────────────┐     ┌─────────────────┐
│ ChapterEmbedding│     │  PodcastAsset   │
│   (Qdrant)      │     │   (Static)      │
└─────────────────┘     └─────────────────┘
```

## Neon Postgres Schema

### Users Table

```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    email_verified BOOLEAN DEFAULT FALSE,
    
    -- Background profile (collected at signup)
    python_level VARCHAR(20) CHECK (python_level IN ('none', 'beginner', 'intermediate', 'advanced')),
    robotics_experience VARCHAR(20) CHECK (robotics_experience IN ('none', 'hobbyist', 'academic', 'professional')),
    math_level VARCHAR(20) CHECK (math_level IN ('basic', 'calculus', 'linear_algebra', 'advanced')),
    
    -- Indexes
    CONSTRAINT users_email_idx UNIQUE (email)
);

CREATE INDEX users_created_at_idx ON users(created_at);
```

**Fields**:
| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| id | UUID | Primary key | Auto-generated |
| email | VARCHAR(255) | User email address | Unique, not null |
| password_hash | VARCHAR(255) | Bcrypt hashed password | Not null |
| created_at | TIMESTAMP | Account creation time | Default NOW() |
| updated_at | TIMESTAMP | Last profile update | Default NOW() |
| email_verified | BOOLEAN | Email verification status | Default false |
| python_level | VARCHAR(20) | Python programming experience | Enum |
| robotics_experience | VARCHAR(20) | Prior robotics background | Enum |
| math_level | VARCHAR(20) | Mathematical background | Enum |

---

### User Preferences Table

```sql
CREATE TABLE user_preferences (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    chapter_slug VARCHAR(100) NOT NULL,
    display_language VARCHAR(5) DEFAULT 'en' CHECK (display_language IN ('en', 'ur')),
    content_depth VARCHAR(20) DEFAULT 'default' CHECK (content_depth IN ('beginner', 'default', 'advanced')),
    last_visited TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    
    CONSTRAINT user_chapter_unique UNIQUE (user_id, chapter_slug)
);

CREATE INDEX user_preferences_user_idx ON user_preferences(user_id);
CREATE INDEX user_preferences_chapter_idx ON user_preferences(chapter_slug);
```

**Fields**:
| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| id | UUID | Primary key | Auto-generated |
| user_id | UUID | Reference to user | Foreign key |
| chapter_slug | VARCHAR(100) | Chapter identifier | e.g., "01-intro-physical-ai" |
| display_language | VARCHAR(5) | Preferred language | 'en' or 'ur' |
| content_depth | VARCHAR(20) | Personalization level | beginner/default/advanced |
| last_visited | TIMESTAMP | Last chapter visit | For analytics |

---

### Chat Sessions Table

```sql
CREATE TABLE chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE SET NULL,
    session_token VARCHAR(255) UNIQUE NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    last_activity TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    is_active BOOLEAN DEFAULT TRUE
);

CREATE INDEX chat_sessions_user_idx ON chat_sessions(user_id);
CREATE INDEX chat_sessions_token_idx ON chat_sessions(session_token);
```

**Fields**:
| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| id | UUID | Primary key | Auto-generated |
| user_id | UUID | Optional user reference | Nullable for anonymous |
| session_token | VARCHAR(255) | Browser session ID | Unique |
| created_at | TIMESTAMP | Session start | Default NOW() |
| last_activity | TIMESTAMP | Last message time | Updated on activity |
| is_active | BOOLEAN | Session status | For cleanup |

---

### Chat Messages Table

```sql
CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES chat_sessions(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant', 'system')),
    content TEXT NOT NULL,
    language VARCHAR(5) DEFAULT 'en' CHECK (language IN ('en', 'ur')),
    selected_text TEXT,
    chapter_context VARCHAR(100),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    
    -- Metadata for RAG
    sources JSONB,
    tokens_used INTEGER
);

CREATE INDEX chat_messages_session_idx ON chat_messages(session_id);
CREATE INDEX chat_messages_created_idx ON chat_messages(created_at);
```

**Fields**:
| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| id | UUID | Primary key | Auto-generated |
| session_id | UUID | Reference to session | Foreign key |
| role | VARCHAR(20) | Message sender | user/assistant/system |
| content | TEXT | Message text | Not null |
| language | VARCHAR(5) | Message language | en/ur |
| selected_text | TEXT | User-highlighted text | Optional |
| chapter_context | VARCHAR(100) | Current chapter | Optional |
| sources | JSONB | RAG source references | Array of chunk IDs |
| tokens_used | INTEGER | OpenAI tokens consumed | For monitoring |

---

### Password Reset Tokens Table

```sql
CREATE TABLE password_reset_tokens (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    token VARCHAR(255) UNIQUE NOT NULL,
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
    used BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX password_reset_user_idx ON password_reset_tokens(user_id);
CREATE INDEX password_reset_token_idx ON password_reset_tokens(token);
```

---

## Qdrant Vector Schema

### Chapter Embeddings Collection

```json
{
  "collection_name": "chapter_embeddings",
  "vectors": {
    "size": 1536,
    "distance": "Cosine"
  },
  "payload_schema": {
    "chapter_slug": "keyword",
    "module": "keyword",
    "section_title": "text",
    "content": "text",
    "chunk_index": "integer",
    "language": "keyword"
  }
}
```

**Vector Point Structure**:
```json
{
  "id": "uuid-v4",
  "vector": [0.123, -0.456, ...],  // 1536 dimensions
  "payload": {
    "chapter_slug": "01-intro-physical-ai",
    "module": "module-1-ros2",
    "section_title": "What is Physical AI?",
    "content": "Physical AI refers to artificial intelligence systems...",
    "chunk_index": 0,
    "language": "en",
    "word_count": 150,
    "has_code": false
  }
}
```

**Indexing Strategy**:
- Chunk by markdown headers (##, ###)
- Maximum 1000 tokens per chunk
- 100 token overlap between chunks
- Separate embeddings for English content only (search returns, translation happens client-side)

---

## Static Content Schema

### Markdown Chapter Structure

```markdown
---
id: chapter-01
title: Introduction to Physical AI & Embodied Intelligence
sidebar_label: 1. Intro to Physical AI
sidebar_position: 1
tags: [physical-ai, embodied-intelligence, introduction]
---

import PodcastPlayer from '@site/src/components/PodcastPlayer';
import PersonalizedContent from '@site/src/components/PersonalizedContent';

<PodcastPlayer 
  englishSrc="/audio/en/chapter-01.mp3"
  urduSrc="/audio/ur/chapter-01.mp3"
/>

## Learning Objectives

By the end of this chapter, you will be able to:
1. Define Physical AI and embodied intelligence
2. Explain the difference between digital and physical AI
3. Identify key components of humanoid robot systems

## Prerequisites

- Basic understanding of artificial intelligence concepts
- Familiarity with Python programming

<PersonalizedContent level="beginner">
## What is Physical AI?

Physical AI refers to...
</PersonalizedContent>

<PersonalizedContent level="advanced">
## Physical AI Overview

Building on foundational AI concepts...
</PersonalizedContent>

## Core Content

[Main chapter content...]

## Exercises

1. **Exercise 1**: [Description]
   <details>
   <summary>Solution</summary>
   [Solution content]
   </details>

## Summary

- Key point 1
- Key point 2
- Key point 3

## References

1. [Reference 1]
2. [Reference 2]
```

---

### Podcast Asset Naming

```
static/audio/
├── en/
│   ├── chapter-01.mp3    # ~10 min, 128kbps
│   ├── chapter-02.mp3
│   └── ...
└── ur/
    ├── chapter-01.mp3
    └── ...
```

**Metadata (derived from filename)**:
| Field | Source | Example |
|-------|--------|---------|
| chapter_id | Filename | chapter-01 |
| language | Directory | en, ur |
| file_path | Full path | /audio/en/chapter-01.mp3 |

---

## State Transitions

### User Authentication States

```
[Anonymous] → SignUp → [Unverified] → VerifyEmail → [Active]
                                                        ↓
[Active] → RequestReset → [ResetPending] → ResetPassword → [Active]
                                                        ↓
[Active] → Deactivate → [Inactive]
```

### Chat Session Lifecycle

```
[New] → CreateSession → [Active] → AddMessage → [Active]
                           ↓
                        30min idle
                           ↓
                       [Expired] → Cleanup → [Deleted]
```

---

## Validation Rules

### User Registration
- Email: Valid format, max 255 chars
- Password: Min 8 chars, at least 1 number, 1 special char
- Background fields: Must be valid enum values

### Chat Messages
- Content: Max 4000 chars
- Selected text: Max 2000 chars
- Language: Must be 'en' or 'ur'

### Preferences
- Chapter slug: Must match existing chapter
- Content depth: Must be valid enum

---

## Migration Scripts

```sql
-- Initial migration: 001_create_tables.sql
-- Run in order: users → user_preferences → chat_sessions → chat_messages → password_reset_tokens
```

Migration files stored in `api/migrations/`.
