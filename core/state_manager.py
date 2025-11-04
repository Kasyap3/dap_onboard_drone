# core/state_manager.py
"""
StateManager: central store of the current drone state.

Responsibilities:
- Keep updated pose, velocity, orientation, battery, mission state.
- Provide thread-safe/async-safe accessors.
- Merge updates from sensors and FCU (telemetry).

API:
- update_from_sensors(sensor_snapshot: dict)
- update_from_fcu(fcu_msg: dict)
- get_state() -> dict
"""

import asyncio
import copy
import logging
import time
from typing import Dict, Any

import numpy as np

logger = logging.getLogger("core.state_manager")
logger.setLevel(logging.INFO)
if not logger.handlers:
    ch = logging.StreamHandler()
    ch.setFormatter(logging.Formatter("[%(asctime)s] %(levelname)s %(message)s"))
    logger.addHandler(ch)


class StateManager:
    def __init__(self):
        # internal state dictionary
        self._state: Dict[str, Any] = {
            "pose": {"lat": None, "lon": None, "alt": None},
            "local_pos": {"x": 0.0, "y": 0.0, "z": 0.0},
            "velocity": {"vx": 0.0, "vy": 0.0, "vz": 0.0},
            "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "battery": {"voltage": None, "soc": None},
            "flight_mode": "DISARMED",
            "mission_stage": None,
            "last_update_ts": time.time()
        }
        self._lock = asyncio.Lock()

    async def update_from_sensors(self, sensor_snapshot: Dict[str, Any]):
        """
        sensor_snapshot: {
           "imu": {...}, "gps": {...}, "baro": {...}, "camera": {...}, "lidar": {...}
        }
        """
        async with self._lock:
            ts = time.time()
            gps = sensor_snapshot.get("gps")
            imu = sensor_snapshot.get("imu")
            baro = sensor_snapshot.get("baro")
            if gps:
                self._state["pose"]["lat"] = float(gps.get("lat"))
                self._state["pose"]["lon"] = float(gps.get("lon"))
                self._state["pose"]["alt"] = float(gps.get("alt", self._state["pose"].get("alt") or 0.0))
            if imu:
                # assuming imu has angular rates and accel; store orientation approx
                self._state["orientation"]["roll"] = float(imu.get("roll", self._state["orientation"]["roll"]))
                self._state["orientation"]["pitch"] = float(imu.get("pitch", self._state["orientation"]["pitch"]))
                self._state["orientation"]["yaw"] = float(imu.get("yaw", self._state["orientation"]["yaw"]))
                self._state["velocity"]["vx"] = float(imu.get("vx", self._state["velocity"]["vx"]))
                self._state["velocity"]["vy"] = float(imu.get("vy", self._state["velocity"]["vy"]))
            if baro and "alt" in baro:
                self._state["pose"]["alt"] = float(baro.get("alt"))
            self._state["last_update_ts"] = ts

    async def update_from_fcu(self, fcu_msg: Dict[str, Any]):
        async with self._lock:
            # fcu_msg is a parsed telemetry or ACK from flight controller
            ts = time.time()
            if "battery" in fcu_msg:
                self._state["battery"].update(fcu_msg["battery"])
            if "mode" in fcu_msg:
                self._state["flight_mode"] = fcu_msg["mode"]
            self._state["last_update_ts"] = ts

    async def set_mission_stage(self, stage: str):
        async with self._lock:
            self._state["mission_stage"] = stage

    async def get_state(self) -> Dict[str, Any]:
        async with self._lock:
            return copy.deepcopy(self._state)

    # Synchronous helper
    def get_state_sync(self) -> Dict[str, Any]:
        """Non-async getter for convenience (not recommended for heavy use)."""
        return copy.deepcopy(self._state)
