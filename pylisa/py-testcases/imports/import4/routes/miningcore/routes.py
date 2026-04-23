# routes/miningcore/routes.py
from fastapi import APIRouter, Depends, Query, HTTPException
from fastapi_cache.decorator import cache
from database import DatabasePool
from utils.logging import logger
from typing import List, Dict, Any
import asyncpg
from datetime import datetime
from utils.calculate import calculate_mining_effort
from utils.cache import MINER_CACHE, POOL_CACHE
from utils.blockchain import get_demurrage_for_block

from .models import PoolStats, Block, Payment, Share
from .queries import (
    POOL_STATS_QUERY, MINER_BLOCKS_QUERY, MINER_PAYMENTS_QUERY,
    CURRENT_SHARES_QUERY, POOL_BLOCKS_QUERY
)
from .utils import (
    format_block_data, format_payment_data, format_share_data,
    get_address_column, handle_db_error, DEMURRAGE_WALLET
)

router = APIRouter()

class MiningCoreException(HTTPException):
    def __init__(self, detail: str):
        super().__init__(status_code=500, detail=detail)
        logger.error(f"MiningCore Error: {detail}")

async def get_connection():
    """Get database connection from pool"""
    pool = await DatabasePool.get_pool()
    async with pool.acquire() as connection:
        yield connection

@router.get("/shares", response_model=List[Share])
@cache(expire=30, key_builder=POOL_CACHE)
async def get_current_shares(conn: asyncpg.Connection = Depends(get_connection)):
    """Get current share counts for all miners"""
    try:
        rows = await conn.fetch(CURRENT_SHARES_QUERY)
        return [format_share_data(row) for row in rows]
    except Exception as e:
        error_msg = handle_db_error("retrieving current shares", e)
        return []