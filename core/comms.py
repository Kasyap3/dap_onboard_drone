# core/comms.py
"""
Unified communication layer for onboard companion computer.

Responsibilities:
- Send telemetry (dict) to ground (via WebSocket/MQTT/MAVLink) -- here: abstracted.
- Receive commands from ground and dispatch to registered callbacks.
- Send commands/ack to flight controller (FCU) via MAVLink/serial (stubbed).
- Provide both sync and async interfaces.

Note: This file provides a solid stub/adapter. Replace transport-specific
implementations (MAVLink, websockets, mqtt) in the TODO sections when integrating
with the actual runtime environment.

Key classes:
- CommsManager
    - register_command_callback(fn)
    - send_telemetry(vehicle_id, payload)
    - send_command_to_fcu(cmd_dict)
    - mock_receive_loop()  # simulates incoming commands (for testing)
"""

import asyncio
import json
import logging
import time
from typing import Callable, Dict, Optional

logger = logging.getLogger("core.comms")
logger.setLevel(logging.INFO)
if not logger.handlers:
    ch = logging.StreamHandler()
    ch.setFormatter(logging.Formatter("[%(asctime)s] %(levelname)s %(message)s"))
    logger.addHandler(ch)


class CommsManager:
    def __init__(self, vehicle_id: str, loop: Optional[asyncio.AbstractEventLoop] = None):
        self.vehicle_id = vehicle_id
        self._cmd_callbacks = []  # functions to call on incoming commands
        self._running = False
        self.loop = loop or asyncio.get_event_loop()
        self._incoming_task = None

        # Placeholders for transport objects (MAVLink, websocket, mqtt)
        self.transport = None  # e.g., websocket client or mqtt client
        self.fcu_transport = None  # serial or pymavlink connection

    # ----------------------
    # Registration / callbacks
    # ----------------------
    def register_command_callback(self, fn: Callable[[Dict], None]):
        """
        Register a callback to handle incoming commands from ground.
        Callback signature: fn(command_dict: Dict) -> None
        """
        self._cmd_callbacks.append(fn)
        logger.debug("Registered command callback: %s", fn)

    # ----------------------
    # Telemetry outgoing
    # ----------------------
    async def send_telemetry(self, payload: Dict) -> bool:
        """
        Send telemetry to ground station. Non-blocking.

        payload: JSON-serializable dict
        returns True on success (stubbed)
        """
        try:
            payload["vehicle_id"] = self.vehicle_id
            payload["timestamp_utc"] = payload.get("timestamp_utc", time.time())
            # TODO: replace print with real websocket/mqtt publish
            logger.debug("Sending telemetry: %s", payload)
            # Simulate network delay
            await asyncio.sleep(0.001)
            # In production: await websocket.send(json.dumps(payload))
            print("[COMMS:TX]", json.dumps(payload))
            return True
        except Exception as e:
            logger.exception("Failed to send telemetry: %s", e)
            return False

    # ----------------------
    # Commands to FCU (flight controller)
    # ----------------------
    async def send_command_to_fcu(self, cmd: Dict) -> bool:
        """
        Send low-level command to the flight controller. This is a stub.
        Replace with pymavlink or serial write in real system.
        """
        try:
            logger.debug("Sending command to FCU: %s", cmd)
            await asyncio.sleep(0.001)
            print("[COMMS:FCU_TX]", json.dumps(cmd))
            return True
        except Exception as e:
            logger.exception("Failed to send command to FCU: %s", e)
            return False

    # ----------------------
    # Internal: simulate incoming commands from ground (for testing)
    # ----------------------
    async def _mock_receive_loop(self, interval_s: float = 2.0):
        """
        Simulated loop that 'receives' command dicts periodically.
        Calls registered callbacks with fake commands.
        """
        self._running = True
        counter = 0
        while self._running:
            await asyncio.sleep(interval_s)
            # Create a fake command occasionally
            counter += 1
            fake_cmd = {
                "cmd_type": "SET_TARGET",
                "seq": counter,
                "target": {"lat": 42.0 + 0.001 * counter, "lon": -71.0, "alt": 50.0}
            }
            logger.info("Mock received command: %s", fake_cmd)
            for cb in self._cmd_callbacks:
                try:
                    # allow callbacks to be sync or async
                    if asyncio.iscoroutinefunction(cb):
                        await cb(fake_cmd)
                    else:
                        cb(fake_cmd)
                except Exception:
                    logger.exception("Command callback failed")
        logger.info("Mock receive loop stopped.")

    def start_mock_receiver(self, interval_s: float = 2.0):
        """Start the simulated receiver in the background loop."""
        if self._incoming_task and not self._incoming_task.done():
            return
        self._incoming_task = self.loop.create_task(self._mock_receive_loop(interval_s))

    def stop_mock_receiver(self):
        self._running = False
        if self._incoming_task:
            self._incoming_task.cancel()
            self._incoming_task = None

    # ----------------------
    # Synchronous wrappers
    # ----------------------
    def send_telemetry_sync(self, payload: Dict) -> bool:
        """Convenience sync wrapper."""
        return self.loop.run_until_complete(self.send_telemetry(payload))
