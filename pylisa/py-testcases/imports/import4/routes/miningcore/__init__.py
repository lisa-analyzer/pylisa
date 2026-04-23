# routes/miningcore/__init__.py
from .routes import router

# Re-export router for use in main app
__all__ = ['router']