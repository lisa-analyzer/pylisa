# routes/miningcore/models.py
from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
from datetime import datetime

class PoolStats(BaseModel):
    id: int
    poolid: str
    connectedminers: int
    poolhashrate: float
    sharespersecond: float
    networkhashrate: float
    networkdifficulty: float
    lastnetworkblocktime: datetime
    blockheight: int
    connectedpeers: int
    created: datetime
    blocks_24h: int
    effort: float = Field(default=0.0)  # Added with default value
    last_block_time: Optional[datetime] = None  # Added to include the block time

    class Config:
        allow_population_by_field_name = True
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }

class Block(BaseModel):
    created: str
    blockheight: int
    effort: float
    reward: float
    confirmationprogress: float
    hasDemurrage: bool = False
    demurrageAmount: float = 0.0
    miner: Optional[str] = None

class Payment(BaseModel):
    created: str
    amount: float
    tx_id: str

class Share(BaseModel):
    miner: str
    shares: float
    last_share: Optional[str]

class TableData(BaseModel):
    rows: List[Dict[str, Any]]