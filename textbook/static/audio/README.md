# Audio Files for Podcast Feature

This directory contains podcast audio files for each chapter in both English and Urdu.

## Directory Structure

```
audio/
├── en/           # English podcasts
│   ├── 01-intro-physical-ai.mp3
│   ├── 02-humanoid-landscape.mp3
│   └── ...
└── ur/           # Urdu podcasts
    ├── 01-intro-physical-ai.mp3
    ├── 02-humanoid-landscape.mp3
    └── ...
```

## Generating Podcasts

### English Podcasts
Use NotebookLM or similar AI tools to generate conversational podcasts:
1. Copy chapter content to NotebookLM
2. Generate audio summary/discussion
3. Download MP3 and rename to match chapter slug
4. Place in `audio/en/` directory

### Urdu Podcasts
Use OpenAI TTS or ElevenLabs:
1. Translate chapter content to Urdu
2. Generate audio using TTS API
3. Place in `audio/ur/` directory

## Naming Convention

Files should be named using the chapter slug:
- `01-intro-physical-ai.mp3`
- `02-humanoid-landscape.mp3`
- `09-gazebo-setup.mp3`
- etc.

## File Requirements

- Format: MP3
- Bitrate: 128kbps or higher
- Sample rate: 44.1kHz
- Channels: Mono or Stereo
