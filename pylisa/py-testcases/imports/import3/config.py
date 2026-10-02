# config.py
from pydantic import BaseSettings
from dotenv import load_dotenv
import os
from typing import Optional

# Load .env file
load_dotenv()

class Settings():
    # Database settings
    DB_HOST: str = os.getenv("DB_HOST", "localhost")
    DB_PORT: int = int(os.getenv("DB_PORT", "5432"))
    DB_USER: str = os.getenv("DB_USER", "")
    DB_PASSWORD: str = os.getenv("DB_PASSWORD", "")
    DB_NAME: str = os.getenv("DB_NAME", "")
    
    # Redis settings
    REDIS_URL: str = os.getenv("REDIS_URL", "redis://localhost")
    
    # Pool settings
    POOL_MIN_SIZE: int = int(os.getenv("POOL_MIN_SIZE", "2"))
    POOL_MAX_SIZE: int = int(os.getenv("POOL_MAX_SIZE", "8"))
    CONNECTION_TIMEOUT: int = int(os.getenv("CONNECTION_TIMEOUT", "10"))
    COMMAND_TIMEOUT: int = int(os.getenv("COMMAND_TIMEOUT", "30"))
    STATEMENT_TIMEOUT: int = int(os.getenv("STATEMENT_TIMEOUT", "20000"))
    CLEANUP_INTERVAL: int = int(os.getenv("CLEANUP_INTERVAL", "300"))
    
    # API settings
    MAX_CONNECTIONS: int = int(os.getenv("MAX_CONNECTIONS", "50"))
    DEBUG: bool = True
    THROTTLE_RATE: int = int(os.getenv("THROTTLE_RATE", "50"))
    ALLOWED_ORIGINS: str = ""  # Comma-separated list of allowed origins in production

    # Monitoring settings
    TELEGRAM_BOT_TOKEN: str = os.getenv("TELEGRAM_BOT_TOKEN", "")
    TELEGRAM_CHAT_ID: str = os.getenv("TELEGRAM_CHAT_ID", "")
    
    # Notification settings
    NOTIFICATION_WINDOW: int = int(os.getenv("NOTIFICATION_WINDOW", "300"))  # 5 minutes
    MAX_SIMILAR_NOTIFICATIONS: int = int(os.getenv("MAX_SIMILAR_NOTIFICATIONS", "3"))
    
    # Health check settings
    HEALTH_CHECK_INTERVAL: int = int(os.getenv("HEALTH_CHECK_INTERVAL", "30"))  # seconds
    MAX_UNHEALTHY_COUNT: int = int(os.getenv("MAX_UNHEALTHY_COUNT", "3"))

    # Ergo Explorer settings
    EXPLORER_API_RETRY_COUNT: int = int(os.getenv("EXPLORER_API_RETRY_COUNT", "3"))
    EXPLORER_API_RETRY_DELAY: int = int(os.getenv("EXPLORER_API_RETRY_DELAY", "3"))
    EXPLORER_API_TIMEOUT: int = int(os.getenv("EXPLORER_API_TIMEOUT", "10"))
    class Config:
        env_file = ".env"
        env_file_encoding = 'utf-8'
        case_sensitive = True

    def get_database_url(self) -> str:
        return f"postgresql://{self.DB_USER}:{self.DB_PASSWORD}@{self.DB_HOST}:{self.DB_PORT}/{self.DB_NAME}"
# Create settings instance
settings = Settings()
