import tiktoken
from openai import OpenAI

from app.config import settings

# Vector size for text-embedding-3-small
VECTOR_SIZE = 1536

client = OpenAI(api_key=settings.OPENAI_API_KEY)


def get_embedding(text: str) -> list[float]:
    """Generate embedding for text using OpenAI."""
    response = client.embeddings.create(model=settings.EMBEDDING_MODEL, input=text)
    return response.data[0].embedding


def count_tokens(text: str, model: str = "gpt-4o-mini") -> int:
    """Count tokens in text."""
    try:
        encoding = tiktoken.encoding_for_model(model)
    except KeyError:
        encoding = tiktoken.get_encoding("cl100k_base")
    return len(encoding.encode(text))


def chunk_text(text: str, max_tokens: int = 500, overlap: int = 50) -> list[str]:
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
