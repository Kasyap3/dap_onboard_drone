# modules/safety/failsafe.py
"""
Failsafe manager.

- Monitors critical signals (battery level, link status, sensor failures) and decides emergency actions:
  e.g., "hover", "land_now", "return_to_home".

API:
- FailsafeManager(thresholds: dict)
- check_and_act(status: dict) -> Optional[dict action]

status expected keys:
  - battery: {"voltage": float, "soc": 0..1}
  - comms: {"rssi": int, "last_heartbeat_s": float}
  - sensors_ok: bool
  - altitude_m: float
  - gps_fix: bool

Return:
  - action dict or None e.g. {"action":"land_now","reason":"low_battery"}
"""

import time
from typing import Optional, Dict


class FailsafeManager:
    def __init__(self, battery_soc_threshold: float = 0.18, heartbeat_timeout_s: float = 5.0):
        self.battery_threshold = battery_soc_threshold
        self.heartbeat_timeout = heartbeat_timeout_s
        self.last_action_time = 0.0
        self.action_cooldown = 2.0  # seconds between actions to avoid flapping

    def check_and_act(self, status: Dict) -> Optional[Dict]:
        """
        Check current status and return an action dict if a failsafe must be triggered.
        """
        now = time.time()
        if now - self.last_action_time < self.action_cooldown:
            return None

        battery = status.get("battery", {})
        soc = battery.get("soc", None)
        if soc is not None and soc <= self.battery_threshold:
            self.last_action_time = now
            return {"action": "land_now", "reason": "low_battery", "soc": soc}

        comms = status.get("comms", {})
        last_hb = comms.get("last_heartbeat_s", 0)
        if last_hb is not None and last_hb > self.heartbeat_timeout:
            # communication lost for too long
            self.last_action_time = now
            return {"action": "hover_and_rth_if_no_link", "reason": "link_lost", "last_heartbeat_s": last_hb}

        sensors_ok = status.get("sensors_ok", True)
        if not sensors_ok:
            self.last_action_time = now
            return {"action": "land_now", "reason": "sensor_failure"}

        # no action required
        return None
