import aiohttp
import json
from datetime import datetime
from typing import List, Dict, Any, Optional, Tuple
import logging
import asyncio
import os

logger = logging.getLogger(__name__)

# Ergo blockchain explorer API base URL
EXPLORER_API_BASE = "https://api.ergoplatform.com/api/v1"
# Local node API endpoint - updated to use Docker host gateway
NODE_API_BASE = "http://host.docker.internal:9053"
# Demurrage wallet address for the mining pool
DEMURRAGE_WALLET = "9fE5o7913CKKe6wvNgM11vULjTuKiopPcvCaj7t2zcJWXM2gcLu"
# Maximum retries for API requests
MAX_RETRIES = 3
# Flag to track if local node is available
LOCAL_NODE_AVAILABLE = True