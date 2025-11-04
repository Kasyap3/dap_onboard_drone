# core/sensors.py
"""
SensorManager: abstract, minimal, and extensible sensor interface.

Responsibilities:
- Detect and register available sensors.
- Provide consistent, timestamped snapshots: imu, gps, baro, camera, lidar.
- Provide async polling loops for sensors (or accept external push updates).

API:
- SensorManager(config: dict)
- register_sensor(name, reader_fn)  # reader_fn returns latest data or awaits it
- get_latest() -> dict (latest snapshot)
- start() / stop() for background reading (if sensors are polled)
"""

import asyncio
import time
import logging
from typing import Callable, Dict, Optional, Any

import numpy as np

logger = logging.getLogger("core.sensors")
logger.setLevel(logging.INFO)
if not logger.handlers:
    ch = logging.StreamHandler()
    ch.setFormatter(logging.Formatter("[%(asctime)s] %(levelname)s %(message)s"))
    logger.addHandler(ch)


class SensorManager:
    def __init__(self, config: Optional[Dict] = None, loop: Optional[asyncio.AbstractEventLoop] = None):
        self.config = config or {}
        self.loop = loop or asyncio.get_event_loop()
        # sensor readers: name -> coroutine or callable
        self._readers: Dict[str, Callable[[], Any]] = {}
        # latest data snapshots
        self._latest: Dict[str, Any] = {}
        self._tasks: Dict[str, asyncio.Task] = {}
        self._running = False

    # ----------------------
    # Registration API
    # ----------------------
    def register_sensor(self, name: str, reader: Callable[[], Any], poll_interval_s: float = 0.1):
        """
        Register a sensor reader.

        reader: callable or coroutine that returns the latest sensor reading.
                Example returns:
                  - imu: {"ax":..,"ay":..,"az":..,"gx":..,"gy":..,"gz":..}
                  - gps: {"lat":..,"lon":..,"alt":.., "fix":bool}
                  - camera: np.ndarray(H,W,3)
        poll_interval_s: how frequently to call reader if it's a sync function.
        """
        self._readers[name] = (reader, poll_interval_s)
        logger.debug("Registered sensor '%s' (interval=%.3fs)", name, poll_interval_s)

    async def _poll_reader_loop(self, name: str):
        reader, interval = self._readers[name]
        while self._running:
            ts = time.time()
            try:
                if asyncio.iscoroutinefunction(reader):
                    val = await reader()
                else:
                    # if it's CPU-bound, consider running in thread pool
                    val = await self.loop.run_in_executor(None, reader)
                self._latest[name] = {"ts": ts, "value": val}
            except Exception:
                logger.exception("Sensor reader '%s' failed", name)
            await asyncio.sleep(interval)

    # ----------------------
    # Control
    # ----------------------
    def start(self):
        """Start polling all registered sensors."""
        if self._running:
            return
        self._running = True
        for name in list(self._readers.keys()):
            task = self.loop.create_task(self._poll_reader_loop(name))
            self._tasks[name] = task
        logger.info("SensorManager started with sensors: %s", list(self._readers.keys()))

    def stop(self):
        self._running = False
        for t in self._tasks.values():
            t.cancel()
        self._tasks.clear()
        logger.info("SensorManager stopped.")

    # ----------------------
    # Accessors
    # ----------------------
    def get_latest(self) -> Dict[str, Any]:
        """
        Returns a shallow copy of the latest snapshot dictionary:
        {
           "imu": {"ts":..., "value": {...}},
           "gps": {"ts":..., "value": {...}},
           ...
        }
        """
        return dict(self._latest)

    def get_latest_value(self, name: str):
        entry = self._latest.get(name)
        return entry["value"] if entry else None

    # ----------------------
    # Utilities: mock/simulated sensors for testing
    # ----------------------
    def register_mock_sensors(self):
        """Register a small set of simulated sensors for local testing."""

        def imu_reader():
            # returns ax, ay, az, gx, gy, gz
            t = time.time()
            return {"ax": 0.0, "ay": 0.0, "az": 9.81, "gx": 0.01, "gy": -0.02, "gz": 0.0, "ts": t}

        def gps_reader():
            # returns lat, lon, alt
            return {"lat": 42.3601, "lon": -71.0589, "alt": 50.0, "fix": True, "ts": time.time()}

        self.register_sensor("imu", imu_reader, poll_interval_s=0.02)
        self.register_sensor("gps", gps_reader, poll_interval_s=0.2)
        logger.info("Mock sensors registered.")
