import os

from dotenv import load_dotenv

load_dotenv()


class Settings:
    # OpenAI
    OPENAI_API_KEY: str = os.getenv("OPENAI_API_KEY", "")
    EMBEDDING_MODEL: str = "text-embedding-3-small"
    CHAT_MODEL: str = "gpt-4o-mini"

    # Qdrant
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    QDRANT_COLLECTION_NAME: str = os.getenv(
        "QDRANT_COLLECTION_NAME", "physical_ai_textbook"
    )

    # Neon Postgres
    DATABASE_URL: str = os.getenv("DATABASE_URL", "")

    # CORS
    FRONTEND_URLS: list = os.getenv(
        "FRONTEND_URLS",
        "http://localhost:3000,https://yasmin191.github.io,https://physical-ai-textbook.vercel.app",
    ).split(",")


settings = Settings()
