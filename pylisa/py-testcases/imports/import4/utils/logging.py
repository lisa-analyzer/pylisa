# utils/logging.py
import logging
import telegram
import asyncio
from functools import partial
from typing import Optional, Dict
from config import settings
import json
from datetime import datetime, timedelta
from redis import asyncio as aioredis
import hashlib

x = 10