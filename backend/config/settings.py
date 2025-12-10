from pydantic_settings import BaseSettings
from typing import Optional

class Settings(BaseSettings):
    # OpenAI settings
    OPENAI_API_KEY: str
    OPENAI_BASE_URL: Optional[str] = "https://api.openai.com/v1"
    OPENAI_MODEL: Optional[str] = "gpt-3.5-turbo"

    # Qdrant settings
    QDRANT_URL: Optional[str] = None
    QDRANT_API_KEY: Optional[str] = None
    QDRANT_COLLECTION_NAME: Optional[str] = "physical_ai_robotics"

    # Neon Postgres settings (for metadata)
    NEON_POSTGRES_URL: Optional[str] = None

    # Application settings
    APP_NAME: str = "Physical AI & Humanoid Robotics RAG API"
    DEBUG: bool = False
    VERSION: str = "1.0.0"
    API_V1_STR: str = "/api/v1"

    class Config:
        env_file = ".env"
        case_sensitive = True

settings = Settings()