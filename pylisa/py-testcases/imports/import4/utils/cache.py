# utils/cache.py
from typing import Any, Callable, Optional
from fastapi import Request
from fastapi_cache import FastAPICache
from fastapi_cache.backends.redis import RedisBackend
from fastapi_cache.coder import Coder
from redis import asyncio as aioredis
import hashlib
import json
from decimal import Decimal
import datetime
import asyncpg
from pydantic import BaseModel

class JSONEncoder(json.JSONEncoder):
    """Custom JSON encoder that handles special types"""
    def default(self, obj):
        if isinstance(obj, BaseModel):
            return obj.dict()
        if isinstance(obj, Decimal):
            return str(obj)
        if isinstance(obj, (datetime.datetime, datetime.date)):
            return obj.isoformat()
        if isinstance(obj, bytes):
            return obj.decode('utf-8')
        try:
            return super().default(obj)
        except TypeError:
            # For any other objects, try to convert to dict if possible
            if hasattr(obj, '__dict__'):
                return obj.__dict__
            return str(obj)  # Last resort: convert to string

class CustomCoder(Coder):
    """Custom coder to handle special types in Redis responses"""
    @classmethod
    def decode(cls, value: Any) -> Any:
        if value is None:
            return None
        if isinstance(value, str):
            return json.loads(value)
        if isinstance(value, bytes):
            return json.loads(value.decode())
        return value

    @classmethod
    def encode(cls, value: Any) -> str:
        return json.dumps(value, cls=JSONEncoder)

class CustomRedisBackend(RedisBackend):
    """Custom Redis backend that handles encoding/decoding properly"""
    async def get(self, key: str) -> Optional[str]:
        value = await self.redis.get(key)
        if value is None:
            return None
        if isinstance(value, bytes):
            return value.decode()
        return value

    async def set(self, key: str, value: str, expire: Optional[int] = None) -> None:
        if isinstance(value, bytes):
            value = value.decode()
        if expire is None:
            await self.redis.set(key, value)
        else:
            await self.redis.set(key, value, ex=expire)

def is_json_serializable(obj: Any) -> bool:
    """Check if an object can be serialized to JSON"""
    try:
        json.dumps(obj, cls=JSONEncoder)
        return True
    except (TypeError, OverflowError):
        return False

def create_cache_key_builder(namespace: str) -> Callable:
    """Creates a cache key builder that safely handles route parameters"""
    def key_builder(
        func: Callable,
        namespace: str = namespace,
        *args: Any,
        **kwargs: Any
    ) -> str:
        # Extract only serializable parameters
        cache_params = {}
        for key, value in kwargs.items():
            # Skip FastAPI dependencies, database connections, and non-serializable objects
            if (
                not isinstance(value, (Request, asyncpg.Connection, asyncpg.pool.PoolConnectionProxy))
                and not key.startswith("_")
                and is_json_serializable(value)
            ):
                cache_params[key] = value

        # Create deterministic string from parameters
        param_str = json.dumps(cache_params, sort_keys=True, cls=JSONEncoder)

        # Create hash of parameters for shorter keys
        param_hash = hashlib.md5(param_str.encode()).hexdigest()[:8]

        # Combine namespace, function name, and parameter hash
        return f"{namespace}:{func.__module__}:{func.__name__}:{param_hash}"

    return key_builder

# Predefined cache key builders for common endpoints
MINER_CACHE = create_cache_key_builder("miner")
POOL_CACHE = create_cache_key_builder("pool")
WORKER_CACHE = create_cache_key_builder("worker")
SETTINGS_CACHE = create_cache_key_builder("settings")
DEMURRAGE_CACHE = create_cache_key_builder("demurrage")

async def setup_cache(redis_url: str):
    """Initialize Redis cache with optimized settings"""
    redis = await aioredis.from_url(
        redis_url,
        encoding="utf8",
        decode_responses=True,
        max_connections=50,
        socket_timeout=5,
        socket_connect_timeout=5,
        socket_keepalive=True,
        health_check_interval=30,
        retry_on_timeout=True
    )

    backend = CustomRedisBackend(redis)
    FastAPICache.init(
        backend,
        prefix="fastapi-cache:",
        key_builder=POOL_CACHE,  # Use pool cache as default
        coder=CustomCoder
    )
    return redis