# modules/navigation/waypoint_follower.py
"""
Waypoint follower.

- Maintain a list of waypoints (lat, lon, alt, desired_speed_mps).
- Given current state, compute next target (lat, lon, alt) and speed, handle arrival.

API:
- WaypointFollower(waypoints: List[Dict])
- set_waypoints(list)
- update(current_state: dict) -> dict next_command

Input current_state:
  {"lat": float, "lon": float, "alt": float, "vx": float, "vy": float}

Output:
  {"target": {"lat","lon","alt"}, "speed_mps": float, "reached": bool, "idx": int}
"""

from typing import List, Dict
import math
import time


def haversine_distance(lat1, lon1, lat2, lon2):
    # returns approx distance in meters (earth radius 6371000 m)
    R = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2.0)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2.0)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


class WaypointFollower:
    def __init__(self, waypoints: List[Dict] = None, acceptance_radius_m: float = 3.0):
        """
        waypoints: list of {"lat", "lon", "alt", "speed_mps" (optional)}
        """
        self.waypoints = waypoints or []
        self.idx = 0
        self.acceptance_radius_m = acceptance_radius_m
        self.last_update = time.time()

    def set_waypoints(self, waypoints: List[Dict]):
        self.waypoints = waypoints
        self.idx = 0

    def _current_wp(self):
        if self.idx < len(self.waypoints):
            return self.waypoints[self.idx]
        return None

    def update(self, current_state: Dict) -> Dict:
        """
        current_state includes lat, lon, alt, speed etc.

        Returns:
            {"target": {...}, "speed_mps":float, "reached": bool, "idx": int}
        """
        wp = self._current_wp()
        if wp is None:
            return {"target": None, "speed_mps": 0.0, "reached": True, "idx": self.idx}

        lat = current_state.get("lat")
        lon = current_state.get("lon")
        alt = current_state.get("alt")

        if lat is None or lon is None:
            # cannot compute -> return current waypoint
            return {"target": wp, "speed_mps": wp.get("speed_mps", 5.0), "reached": False, "idx": self.idx}

        dist = haversine_distance(lat, lon, wp["lat"], wp["lon"])
        reached = dist <= self.acceptance_radius_m

        if reached:
            self.idx += 1
            next_wp = self._current_wp()
            return {"target": next_wp, "speed_mps": (next_wp.get("speed_mps", 5.0) if next_wp else 0.0),
                    "reached": True, "idx": self.idx - 1}
        else:
            return {"target": wp, "speed_mps": wp.get("speed_mps", 5.0), "reached": False, "idx": self.idx}
