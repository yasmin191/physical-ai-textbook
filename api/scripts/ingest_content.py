"""
Content ingestion script for indexing textbook chapters into Qdrant.

Usage:
    python -m scripts.ingest_content [--chapter CHAPTER_SLUG]
"""

import asyncio
import os
import re
import sys
from pathlib import Path
from typing import Optional

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from dotenv import load_dotenv

load_dotenv()

from services.embedding_service import get_embedding_service
from services.qdrant_service import get_qdrant_service


# Module mapping
MODULES = {
    "module-1-ros2": "Module 1: ROS 2 Fundamentals",
    "module-2-simulation": "Module 2: Simulation",
    "module-3-isaac": "Module 3: NVIDIA Isaac",
    "module-4-vla": "Module 4: Vision-Language-Action",
    "appendices": "Appendices",
}


def extract_frontmatter(content: str) -> tuple[dict, str]:
    """Extract YAML frontmatter from markdown."""
    frontmatter = {}
    body = content

    if content.startswith("---"):
        parts = content.split("---", 2)
        if len(parts) >= 3:
            # Parse simple YAML
            yaml_content = parts[1].strip()
            for line in yaml_content.split("\n"):
                if ":" in line:
                    key, value = line.split(":", 1)
                    frontmatter[key.strip()] = value.strip().strip('"').strip("'")
            body = parts[2]

    return frontmatter, body


def get_chapter_title(content: str, frontmatter: dict) -> str:
    """Extract chapter title from content or frontmatter."""
    if "title" in frontmatter:
        return frontmatter["title"]

    # Look for first H1
    match = re.search(r"^#\s+(.+)$", content, re.MULTILINE)
    if match:
        return match.group(1).strip()

    return "Untitled Chapter"


async def ingest_chapter(
    file_path: Path,
    module_name: str,
    embedding_service,
    qdrant_service,
) -> int:
    """Ingest a single chapter file."""
    print(f"  Processing: {file_path.name}")

    # Read file
    content = file_path.read_text(encoding="utf-8")

    # Extract frontmatter and body
    frontmatter, body = extract_frontmatter(content)

    # Get chapter metadata
    chapter_slug = file_path.stem
    chapter_title = get_chapter_title(body, frontmatter)

    # Chunk the content
    chunks = embedding_service.chunk_markdown(
        content=body,
        chapter_slug=chapter_slug,
        chapter_title=chapter_title,
        module=module_name,
    )

    if not chunks:
        print(f"    No chunks created (empty content?)")
        return 0

    print(f"    Created {len(chunks)} chunks")

    # Generate embeddings
    embeddings = await embedding_service.embed_chunks(chunks)
    print(f"    Generated {len(embeddings)} embeddings")

    # Prepare chunks for Qdrant
    chunk_dicts = [
        {
            "id": chunk.id,
            "text": chunk.text,
            "chapter_slug": chunk.chapter_slug,
            "section_title": chunk.section_title,
            "chapter_title": chunk.chapter_title,
            "module": chunk.module,
        }
        for chunk in chunks
    ]

    # Upsert to Qdrant
    count = await qdrant_service.upsert_chunks(chunk_dicts, embeddings)
    print(f"    Indexed {count} chunks to Qdrant")

    return count


async def ingest_module(
    module_dir: Path,
    module_name: str,
    embedding_service,
    qdrant_service,
) -> int:
    """Ingest all chapters in a module."""
    total_chunks = 0

    # Get all markdown files (exclude category files)
    md_files = sorted(
        [f for f in module_dir.glob("*.md") if not f.name.startswith("_")]
    )

    for md_file in md_files:
        chunks = await ingest_chapter(
            md_file,
            module_name,
            embedding_service,
            qdrant_service,
        )
        total_chunks += chunks

    return total_chunks


async def ingest_all(
    docs_dir: Path,
    chapter_filter: Optional[str] = None,
) -> dict:
    """Ingest all textbook content."""
    embedding_service = get_embedding_service()
    qdrant_service = get_qdrant_service()

    # Ensure collection exists
    await qdrant_service.ensure_collection()

    stats = {
        "modules_processed": 0,
        "chapters_processed": 0,
        "total_chunks": 0,
    }

    # Process each module
    for module_slug, module_name in MODULES.items():
        module_dir = docs_dir / module_slug

        if not module_dir.exists():
            print(f"Skipping {module_slug} (not found)")
            continue

        print(f"\nProcessing {module_name}...")

        if chapter_filter:
            # Process single chapter
            chapter_file = module_dir / f"{chapter_filter}.md"
            if chapter_file.exists():
                chunks = await ingest_chapter(
                    chapter_file,
                    module_name,
                    embedding_service,
                    qdrant_service,
                )
                stats["chapters_processed"] += 1
                stats["total_chunks"] += chunks
        else:
            # Process all chapters in module
            chunks = await ingest_module(
                module_dir,
                module_name,
                embedding_service,
                qdrant_service,
            )
            stats["modules_processed"] += 1
            stats["total_chunks"] += chunks

    # Also process intro.md if it exists
    intro_file = docs_dir / "intro.md"
    if intro_file.exists() and not chapter_filter:
        print("\nProcessing intro.md...")
        chunks = await ingest_chapter(
            intro_file,
            "Introduction",
            embedding_service,
            qdrant_service,
        )
        stats["total_chunks"] += chunks

    return stats


async def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="Ingest textbook content into Qdrant")
    parser.add_argument(
        "--chapter",
        type=str,
        help="Specific chapter slug to ingest (e.g., '01-intro-physical-ai')",
    )
    parser.add_argument(
        "--docs-dir",
        type=str,
        default="../textbook/docs",
        help="Path to docs directory",
    )

    args = parser.parse_args()

    # Resolve docs directory
    script_dir = Path(__file__).parent
    docs_dir = (script_dir / args.docs_dir).resolve()

    if not docs_dir.exists():
        print(f"Error: Docs directory not found: {docs_dir}")
        sys.exit(1)

    print(f"Ingesting content from: {docs_dir}")
    print("=" * 50)

    stats = await ingest_all(docs_dir, args.chapter)

    print("\n" + "=" * 50)
    print("Ingestion complete!")
    print(f"  Modules processed: {stats['modules_processed']}")
    print(f"  Total chunks indexed: {stats['total_chunks']}")

    # Show collection info
    qdrant_service = get_qdrant_service()
    info = await qdrant_service.get_collection_info()
    print(f"\nQdrant collection info:")
    print(f"  Collection: {info.get('name', 'unknown')}")
    print(f"  Total points: {info.get('points_count', 'unknown')}")


if __name__ == "__main__":
    asyncio.run(main())
