"""
Script to ingest textbook content into Qdrant vector database.
Run this script after setting up your environment variables.
"""

import os
import re
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from dotenv import load_dotenv

load_dotenv()

from app.services.qdrant_service import ensure_collection_exists, add_document
from app.services.embedding_service import chunk_text

# Path to textbook docs
DOCS_PATH = Path(__file__).parent.parent.parent / "textbook" / "docs"


def extract_frontmatter(content: str) -> tuple[dict, str]:
    """Extract frontmatter and content from markdown."""
    frontmatter = {}

    if content.startswith("---"):
        parts = content.split("---", 2)
        if len(parts) >= 3:
            fm_text = parts[1].strip()
            for line in fm_text.split("\n"):
                if ":" in line:
                    key, value = line.split(":", 1)
                    frontmatter[key.strip()] = value.strip().strip("\"'")
            content = parts[2].strip()

    return frontmatter, content


def clean_markdown(content: str) -> str:
    """Remove markdown syntax for cleaner text."""
    # Remove code blocks
    content = re.sub(r"```[\s\S]*?```", "[CODE BLOCK]", content)
    # Remove inline code
    content = re.sub(r"`[^`]+`", "", content)
    # Remove images
    content = re.sub(r"!\[.*?\]\(.*?\)", "", content)
    # Remove links but keep text
    content = re.sub(r"\[([^\]]+)\]\([^\)]+\)", r"\1", content)
    # Remove HTML tags
    content = re.sub(r"<[^>]+>", "", content)
    # Remove multiple newlines
    content = re.sub(r"\n{3,}", "\n\n", content)

    return content.strip()


def extract_sections(content: str) -> list[dict]:
    """Extract sections from markdown content."""
    sections = []
    current_section = {"title": "Introduction", "content": ""}

    lines = content.split("\n")

    for line in lines:
        # Check for headers
        if line.startswith("## "):
            if current_section["content"].strip():
                sections.append(current_section)
            current_section = {"title": line[3:].strip(), "content": ""}
        elif line.startswith("### "):
            if current_section["content"].strip():
                sections.append(current_section)
            current_section = {"title": line[4:].strip(), "content": ""}
        else:
            current_section["content"] += line + "\n"

    # Add last section
    if current_section["content"].strip():
        sections.append(current_section)

    return sections


def process_file(file_path: Path, chapter_name: str):
    """Process a single markdown file."""
    print(f"Processing: {file_path.name}")

    content = file_path.read_text(encoding="utf-8")
    frontmatter, content = extract_frontmatter(content)

    title = frontmatter.get("title", chapter_name)

    # Clean the markdown
    cleaned_content = clean_markdown(content)

    # Extract sections
    sections = extract_sections(cleaned_content)

    # Process each section
    for section in sections:
        section_content = section["content"].strip()
        if len(section_content) < 50:  # Skip very short sections
            continue

        # Chunk the content
        chunks = chunk_text(section_content, max_tokens=400, overlap=50)

        for i, chunk in enumerate(chunks):
            if len(chunk.strip()) < 30:
                continue

            doc_id = add_document(
                content=chunk,
                chapter=title,
                section=section["title"],
                metadata={
                    "file": file_path.name,
                    "chunk_index": i,
                    "total_chunks": len(chunks),
                },
            )
            print(
                f"  Added chunk {i + 1}/{len(chunks)} from section: {section['title'][:30]}..."
            )


def ingest_all():
    """Ingest all textbook content."""
    print("Starting content ingestion...")

    # Ensure collection exists
    ensure_collection_exists()

    # Process all module directories
    module_dirs = [
        "module-1-ros2",
        "module-2-simulation",
        "module-3-isaac",
        "module-4-vla",
        "appendices",
    ]

    total_files = 0

    for module_dir in module_dirs:
        module_path = DOCS_PATH / module_dir
        if not module_path.exists():
            print(f"Skipping {module_dir} (not found)")
            continue

        print(f"\n=== Processing {module_dir} ===")

        for md_file in sorted(module_path.glob("*.md")):
            if md_file.name.startswith("_"):
                continue

            chapter_name = md_file.stem.replace("-", " ").title()
            process_file(md_file, chapter_name)
            total_files += 1

    print(f"\n=== Ingestion complete! Processed {total_files} files ===")

    print(f"\n=== Ingestion complete! Processed {total_files} files ===")

if __name__ == "__main__":
    ingest_all()
