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

# routes/miningcore/routes.py

# ... other imports ...
from utils.calculate import calculate_mining_effort
from utils.logging import logger

@router.get("/poolstats")
@cache(expire=60, key_builder=POOL_CACHE)
async def get_pool_stats(conn: asyncpg.Connection = Depends(get_connection)):
    """Get current pool statistics"""
    try:
        query = """
            WITH latest_stats AS (
                SELECT *
                FROM poolstats
                ORDER BY created DESC
                LIMIT 1
            ),
            pool_blocks AS (
                SELECT COUNT(*) as blocks_24h
                FROM blocks
                WHERE created >= NOW() - INTERVAL '24 hours'
            ),
            total_blocks AS (
                SELECT COUNT(*) as total_blocks_count
                FROM blocks
            ),
            last_block AS (
                SELECT created as last_block_time
                FROM blocks
                WHERE confirmationprogress = 1
                ORDER BY created DESC
                LIMIT 1
            )
            SELECT
                ls.id,
                ls.poolid,
                ls.connectedminers,
                ls.poolhashrate,
                ls.sharespersecond,
                ls.networkhashrate,
                ls.networkdifficulty,
                ls.lastnetworkblocktime,
                ls.blockheight,
                ls.connectedpeers,
                ls.created,
                pb.blocks_24h,
                tb.total_blocks_count,
                lb.last_block_time
            FROM latest_stats ls
            CROSS JOIN pool_blocks pb
            CROSS JOIN total_blocks tb
            CROSS JOIN last_block lb
        """

        result = await conn.fetch(query)
        if not result:
            logger.error("No pool stats found")
            return {"error": "No pool stats available"}

        stats = dict(result[0])

        # Log values used for effort calculation
        logger.info(f"Network difficulty: {stats['networkdifficulty']}")
        logger.info(f"Network hashrate: {stats['networkhashrate']}")
        logger.info(f"Pool hashrate: {stats['poolhashrate']}")
        logger.info(f"Last block time: {stats.get('last_block_time')}")
        logger.info(f"Total blocks found: {stats['total_blocks_count']}")
        logger.info(f"Blocks found in last 24h: {stats['blocks_24h']}")

        # Calculate effort
        if stats.get('last_block_time'):
            effort = calculate_mining_effort(
                network_difficulty=float(stats['networkdifficulty']),
                network_hashrate=float(stats['networkhashrate']),
                hashrate=float(stats['poolhashrate']),
                last_block_timestamp=stats['last_block_time'].isoformat()
            )
            logger.info(f"Calculated effort: {effort}")
            stats['effort'] = effort
        else:
            logger.warning("No last block time found, setting effort to 0")
            stats['effort'] = 0.0

        # Format response to match frontend expected structure
        formatted_response = {
            "poolId": stats['poolid'],
            "poolStats": {
                "connectedMiners": stats['connectedminers'],
                "poolHashrate": float(stats['poolhashrate']),
                "sharesPerSecond": float(stats['sharespersecond'])
            },
            "networkStats": {
                "networkHashrate": float(stats['networkhashrate']),
                "networkDifficulty": float(stats['networkdifficulty']),
                "lastNetworkBlockTime": stats['lastnetworkblocktime'].isoformat() if stats['lastnetworkblocktime'] else None,
                "blockHeight": stats['blockheight'],
                "connectedPeers": stats['connectedpeers']
            },
            "effort": float(stats['effort']),
            "blocks24h": stats['blocks_24h'],
            "totalBlocks": stats['total_blocks_count'],
            "lastBlockTime": stats['last_block_time'].isoformat() if stats['last_block_time'] else None,
            "created": stats['created'].isoformat() if stats['created'] else None
        }

        return formatted_response
    except Exception as e:
        logger.error(f"Error getting pool stats: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/blocks/{address}", response_model=List[Block])
@cache(expire=30, key_builder=MINER_CACHE)
async def get_miner_blocks(
    address: str,
    conn: asyncpg.Connection = Depends(get_connection),
    limit: int = Query(100, ge=1, le=1000)
):
    """Get blocks found by a specific miner"""
    try:
        rows = await conn.fetch(MINER_BLOCKS_QUERY, address, limit)

        blocks = []
        for row in rows:
            effort = float(row['stored_effort']) if row['stored_effort'] is not None else calculate_mining_effort(
                row['networkdifficulty'],
                row['networkhashrate'],
                row['current_hashrate'] if row['current_hashrate'] else 0,
                row['created'].isoformat()
            )

            # Check for demurrage info
            block_height = row['blockheight']
            has_demurrage, demurrage_amount = await get_demurrage_for_block(block_height, DEMURRAGE_WALLET)

            blocks.append(format_block_data(
                row=row,
                effort=effort,
                has_demurrage=has_demurrage,
                demurrage_amount=demurrage_amount
            ))

        return blocks
    except Exception as e:
        error_msg = handle_db_error(f"retrieving blocks for miner {address}", e)
        raise MiningCoreException(error_msg)

@router.get("/blocks", response_model=List[Block])
@cache(expire=30, key_builder=POOL_CACHE)
async def get_pool_blocks(
    conn: asyncpg.Connection = Depends(get_connection),
    limit: int = Query(100, ge=1, le=1000)
):
    """Get all blocks found by the pool"""
    try:
        rows = await conn.fetch(POOL_BLOCKS_QUERY, limit)

        blocks = []
        for row in rows:
            effort = float(row['stored_effort']) if row['stored_effort'] is not None else 0.0

            # Check for demurrage info
            block_height = row['blockheight']
            has_demurrage, demurrage_amount = await get_demurrage_for_block(block_height, DEMURRAGE_WALLET)

            # Debug miner information
            miner_address = row.get('miner')
            if miner_address:
                logger.info(f"Block {block_height} has miner: {miner_address}")
            else:
                logger.info(f"Block {block_height} has NO miner data")

            formatted_block = format_block_data(
                row=row,
                effort=effort,
                has_demurrage=has_demurrage,
                demurrage_amount=demurrage_amount
            )

            # Ensure miner is explicitly added even if None
            if 'miner' not in formatted_block:
                formatted_block['miner'] = miner_address

            blocks.append(formatted_block)

        return blocks
    except Exception as e:
        error_msg = handle_db_error("retrieving pool blocks", e)
        raise MiningCoreException(error_msg)

@router.get("/payments/{address}", response_model=List[Payment])
@cache(expire=60, key_builder=MINER_CACHE)
async def get_miner_payments(
    address: str,
    conn: asyncpg.Connection = Depends(get_connection),
    limit: int = Query(100, ge=1, le=1000)
):
    """Get payment history for a specific miner"""
    try:
        rows = await conn.fetch(MINER_PAYMENTS_QUERY, address, limit)
        return [format_payment_data(row) for row in rows]
    except Exception as e:
        error_msg = handle_db_error(f"retrieving payments for miner {address}", e)
        raise MiningCoreException(error_msg)

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

@router.get("/{table_name}")
@cache(expire=60, key_builder=POOL_CACHE)
async def get_table_data(
    table_name: str,
    conn: asyncpg.Connection = Depends(get_connection)
):
    """Get all data from a specific table"""
    try:
        result = await conn.fetch(f"SELECT * FROM {table_name}")
        # Return the rows directly without wrapping
        return [dict(row) for row in result]
    except Exception as e:
        error_msg = handle_db_error(f"retrieving data from {table_name}", e)
        raise HTTPException(status_code=500, detail="Database error")

@router.get("/{table_name}/{address}")
@cache(expire=60, key_builder=MINER_CACHE)
async def get_filtered_table_data(
    table_name: str,
    address: str,
    conn: asyncpg.Connection = Depends(get_connection),
    limit: int = Query(100, ge=1, le=1000)
) -> List[Dict[str, Any]]:
    """Get filtered data from a specific table for a given address"""
    try:
        # Check if table exists
        table_exists = await conn.fetchval(
            "SELECT EXISTS (SELECT FROM information_schema.tables WHERE table_name = $1)",
            table_name
        )
        if not table_exists:
            raise HTTPException(status_code=404, detail=f"Table '{table_name}' not found")

        address_column = get_address_column(table_name)
        query = f"SELECT * FROM {table_name} WHERE {address_column} = $1 LIMIT $2"
        rows = await conn.fetch(query, address, limit)

        # Return the rows directly without wrapping
        result = [dict(row) for row in rows]
        logger.info(f"Retrieved {len(result)} rows from table {table_name} for address {address}")
        return result
    except Exception as e:
        error_msg = handle_db_error(f"retrieving filtered data from {table_name}", e)
        raise HTTPException(status_code=500, detail="Database error")