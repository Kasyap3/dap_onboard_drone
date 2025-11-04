# modules/safety/watchdog.py
"""
Heartbeat watchdog.

- Register heartbeats from ground or internal modules.
- Check periodic health; if missing, set flagged state.

API:
- Watchdog(timeout_s=2.0)
- kick(source_id)
- check() -> List[str] missing sources
"""

import time
from typing import Dict, List


class Watchdog:
    def __init__(self, timeout_s: float = 2.0):
        self.timeout = float(timeout_s)
        self.sources: Dict[str, float] = {}

    def register_source(self, source_id: str):
        self.sources[source_id] = time.time()

    def kick(self, source_id: str):
        self.sources[source_id] = time.time()

    def last_seen(self, source_id: str) -> float:
        return self.sources.get(source_id, 0.0)

    def check(self) -> List[str]:
        now = time.time()
        missing = []
        for src, ts in self.sources.items():
            if now - ts > self.timeout:
                missing.append(src)
        return missing
