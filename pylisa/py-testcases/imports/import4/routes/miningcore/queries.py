# routes/miningcore/queries.py

POOL_STATS_QUERY = """
    WITH latest_stats AS (
        SELECT 
            id,
            poolid,
            connectedminers,
            poolhashrate,
            sharespersecond,
            networkhashrate,
            networkdifficulty,
            lastnetworkblocktime,
            blockheight,
            connectedpeers,
            created
        FROM poolstats
        ORDER BY created DESC
        LIMIT 1
    ),
    pool_blocks AS (
        SELECT COUNT(*) as blocks_24h
        FROM blocks
        WHERE created >= NOW() - INTERVAL '24 hours'
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
        pb.blocks_24h
    FROM latest_stats ls
    CROSS JOIN pool_blocks pb
"""

MINER_BLOCKS_QUERY = """
    WITH pool_stats AS (
        SELECT networkdifficulty, networkhashrate
        FROM poolstats
        ORDER BY created DESC
        LIMIT 1
    ),
    miner_stats AS (
        SELECT hashrate
        FROM minerstats
        WHERE miner = $1
        ORDER BY created DESC
        LIMIT 1
    )
    SELECT 
        b.created,
        b.blockheight,
        b.effort as stored_effort,
        b.reward,
        b.confirmationprogress,
        p.networkdifficulty,
        p.networkhashrate,
        ms.hashrate as current_hashrate
    FROM blocks b
    CROSS JOIN pool_stats p
    LEFT JOIN miner_stats ms ON true
    WHERE b.miner = $1
    ORDER BY b.created DESC
    LIMIT $2
"""

POOL_BLOCKS_QUERY = """
    WITH pool_stats AS (
        SELECT networkdifficulty, networkhashrate
        FROM poolstats
        ORDER BY created DESC
        LIMIT 1
    )
    SELECT 
        b.created,
        b.blockheight,
        b.effort as stored_effort,
        b.reward,
        b.confirmationprogress,
        p.networkdifficulty,
        p.networkhashrate,
        b.miner
    FROM blocks b
    CROSS JOIN pool_stats p
    ORDER BY b.created DESC
    LIMIT $1
"""

MINER_PAYMENTS_QUERY = """
    SELECT 
        created,
        amount,
        transactionconfirmationdata
    FROM payments
    WHERE address = $1
    ORDER BY created DESC
    LIMIT $2
"""

CURRENT_SHARES_QUERY = """
    WITH current_round AS (
        SELECT MAX(created) as last_block
        FROM blocks
        WHERE confirmationprogress >= 1
    )
    SELECT 
        m.miner,
        COALESCE(SUM(m.sharespersecond), 0) as shares,
        MAX(m.created) as last_share
    FROM minerstats m
    CROSS JOIN current_round cr
    WHERE m.created > cr.last_block
    GROUP BY m.miner
"""

ADDRESS_COLUMNS = {
    "shares": "miner",
    "balances": "address",
    "balance_changes": "address",
    "payments": "address",
    "minerstats": "miner",
    "blocks": "miner",
}