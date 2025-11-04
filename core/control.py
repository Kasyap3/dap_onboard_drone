# core/control.py
"""
Control interface.

Responsibilities:
- Provide high-level actions: set_velocity, set_attitude, takeoff, land, hover.
- Convert high-level commands into FCU-friendly messages and send via CommsManager.

APIs:
- ControlInterface(comms: CommsManager, state_manager: StateManager)
- async set_velocity(vx, vy, vz, yaw_rate)
- async takeoff(target_alt_m)
- async land()
- emergency_stop()
"""

import asyncio
import logging
from typing import Dict, Optional

logger = logging.getLogger("core.control")
logger.setLevel(logging.INFO)
if not logger.handlers:
    ch = logging.StreamHandler()
    ch.setFormatter(logging.Formatter("[%(asctime)s] %(levelname)s %(message)s"))
    logger.addHandler(ch)


class ControlInterface:
    def __init__(self, comms, state_manager, default_altitude_m: float = 10.0):
        """
        comms: instance of CommsManager (from core.comms)
        state_manager: instance of StateManager
        """
        self.comms = comms
        self.state = state_manager
        self.default_alt = default_altitude_m
        self._last_cmd_seq = 0

    def _next_seq(self):
        self._last_cmd_seq += 1
        return self._last_cmd_seq

    async def set_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0) -> bool:
        """
        Set velocity in body or local frame (define convention in system).
        """
        cmd = {
            "cmd_type": "SET_VELOCITY",
            "seq": self._next_seq(),
            "vx": float(vx),
            "vy": float(vy),
            "vz": float(vz),
            "yaw_rate": float(yaw_rate),
            "timestamp": asyncio.get_event_loop().time()
        }
        logger.debug("set_velocity -> %s", cmd)
        return await self.comms.send_command_to_fcu(cmd)

    async def set_attitude(self, roll: float, pitch: float, yaw_rate: float, throttle: float) -> bool:
        cmd = {
            "cmd_type": "SET_ATTITUDE",
            "seq": self._next_seq(),
            "roll": float(roll),
            "pitch": float(pitch),
            "yaw_rate": float(yaw_rate),
            "throttle": float(throttle),
            "timestamp": asyncio.get_event_loop().time()
        }
        logger.debug("set_attitude -> %s", cmd)
        return await self.comms.send_command_to_fcu(cmd)

    async def takeoff(self, target_alt_m: Optional[float] = None) -> bool:
        tgt = target_alt_m if target_alt_m is not None else self.default_alt
        cmd = {"cmd_type": "TAKEOFF", "seq": self._next_seq(), "target_alt_m": float(tgt)}
        logger.info("Issuing takeoff to %.1f m", tgt)
        return await self.comms.send_command_to_fcu(cmd)

    async def land(self) -> bool:
        cmd = {"cmd_type": "LAND", "seq": self._next_seq()}
        logger.info("Issuing land command")
        return await self.comms.send_command_to_fcu(cmd)

    async def hold_position(self) -> bool:
        cmd = {"cmd_type": "HOLD", "seq": self._next_seq()}
        logger.info("Issuing hold/hover command")
        return await self.comms.send_command_to_fcu(cmd)

    async def emergency_stop(self) -> bool:
        cmd = {"cmd_type": "EMERGENCY_STOP", "seq": self._next_seq()}
        logger.critical("Issuing EMERGENCY STOP")
        return await self.comms.send_command_to_fcu(cmd)
