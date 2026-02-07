"""
Script to ingest textbook content into Qdrant vector database.
Run this script locally after setting up your environment variables.

Usage:
    cd chatbot-api
    pip install openai qdrant-client python-dotenv tiktoken
    python scripts/ingest_content.py
"""

import os
import re
from pathlib import Path

import tiktoken
from dotenv import load_dotenv
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Load environment variables
load_dotenv()

# Configuration
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")
QDRANT_URL = os.getenv("QDRANT_URL", "")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "")
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_textbook")
EMBEDDING_MODEL = "text-embedding-3-small"
VECTOR_SIZE = 1536

# Initialize clients
openai_client = OpenAI(api_key=OPENAI_API_KEY)
qdrant_client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
    timeout=60,  # Increase timeout to 60 seconds
)

# Path to textbook docs
DOCS_PATH = Path(__file__).parent.parent.parent / "textbook" / "docs"


def get_embedding(text: str) -> list[float]:
    """Generate embedding for text using OpenAI."""
    response = openai_client.embeddings.create(model=EMBEDDING_MODEL, input=text)
    return response.data[0].embedding


def count_tokens(text: str) -> int:
    """Count tokens in text."""
    try:
        encoding = tiktoken.encoding_for_model("gpt-4o-mini")
    except KeyError:
        encoding = tiktoken.get_encoding("cl100k_base")
    return len(encoding.encode(text))


def chunk_text(text: str, max_tokens: int = 400, overlap: int = 50) -> list[str]:
    """Split text into chunks with overlap."""
    try:
        encoding = tiktoken.encoding_for_model("gpt-4o-mini")
    except KeyError:
        encoding = tiktoken.get_encoding("cl100k_base")

    tokens = encoding.encode(text)
    chunks = []

    start = 0
    while start < len(tokens):
        end = start + max_tokens
        chunk_tokens = tokens[start:end]
        chunk_text = encoding.decode(chunk_tokens)
        chunks.append(chunk_text)
        start = end - overlap

    return chunks


def ensure_collection_exists():
    """Create collection if it doesn't exist."""
    try:
        collections = qdrant_client.get_collections().collections
        collection_names = [c.name for c in collections]

        if COLLECTION_NAME not in collection_names:
            qdrant_client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=models.VectorParams(
                    size=VECTOR_SIZE,
                    distance=models.Distance.COSINE,
                ),
            )
            print(f"Created collection: {COLLECTION_NAME}")
        else:
            print(f"Collection {COLLECTION_NAME} already exists")
    except Exception as e:
        print(f"Error checking/creating collection: {e}")
        raise


def add_document(
    content: str, chapter: str, section: str = "", metadata: dict = None
) -> str:
    """Add a document chunk to Qdrant."""
    import uuid

    doc_id = str(uuid.uuid4())
    embedding = get_embedding(content)

    payload = {
        "content": content,
        "chapter": chapter,
        "section": section or "",
        **(metadata or {}),
    }

    qdrant_client.upsert(
        collection_name=COLLECTION_NAME,
        points=[
            models.PointStruct(
                id=doc_id,
                vector=embedding,
                payload=payload,
            )
        ],
    )

    return doc_id


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

    chunks_added = 0

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

            try:
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
                chunks_added += 1
            except Exception as e:
                print(f"  Error adding chunk: {e}")

    print(f"  Added {chunks_added} chunks from {len(sections)} sections")
    return chunks_added


def ingest_all():
    """Ingest all textbook content."""
    print("=" * 60)
    print("Starting content ingestion into Qdrant")
    print("=" * 60)

    # Check configuration
    if not OPENAI_API_KEY:
        print("ERROR: OPENAI_API_KEY not set")
        return
    if not QDRANT_URL:
        print("ERROR: QDRANT_URL not set")
        return
    if not QDRANT_API_KEY:
        print("ERROR: QDRANT_API_KEY not set")
        return

    print(f"Qdrant URL: {QDRANT_URL}")
    print(f"Collection: {COLLECTION_NAME}")
    print(f"Docs path: {DOCS_PATH}")
    print()

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
    total_chunks = 0

    for module_dir in module_dirs:
        module_path = DOCS_PATH / module_dir
        if not module_path.exists():
            print(f"Skipping {module_dir} (not found)")
            continue

        print(f"\n{'=' * 40}")
        print(f"Processing {module_dir}")
        print("=" * 40)

        for md_file in sorted(module_path.glob("*.md")):
            if md_file.name.startswith("_"):
                continue

            chapter_name = md_file.stem.replace("-", " ").title()
            chunks = process_file(md_file, chapter_name)
            total_chunks += chunks
            total_files += 1

    print(f"\n{'=' * 60}")
    print(f"Ingestion complete!")
    print(f"Processed {total_files} files, added {total_chunks} chunks")
    print("=" * 60)


if __name__ == "__main__":
    ingest_all()
