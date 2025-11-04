# modules/navigation/local_planner.py
"""
Local planner (short-horizon obstacle-aware planner).

- Given current pose and a next waypoint, and a list of obstacles (relative positions and radii),
  compute a short-horizon velocity setpoint or small waypoint that steers around obstacles.

API:
- LocalPlanner()
- plan(current_pose: dict, goal: dict, obstacles: List[Dict]) -> dict

Inputs:
  - current_pose: {"lat","lon","alt","vx","vy","vz","yaw"}
  - goal: {"lat","lon","alt"}
  - obstacles: [{"position":{"rel_x","rel_y"}, "radius": float, "confidence": float}, ...]

Output:
  - short-term command: {"vx": float, "vy": float, "vz": float, "yaw_rate": float}
  - or small intermediate waypoint: {"lat","lon","alt"}
"""

from typing import List, Dict
import math


class LocalPlanner:
    def __init__(self, max_speed_mps=8.0, min_clearance_m=2.0):
        self.max_speed = max_speed_mps
        self.min_clearance = min_clearance_m

    def _bearing_distance_to_goal(self, cur_lat, cur_lon, goal_lat, goal_lon):
        # returns (bearing_rad, distance_m)
        # Use simple equirectangular projection for short distances
        R = 6371000.0
        dlat = math.radians(goal_lat - cur_lat)
        dlon = math.radians(goal_lon - cur_lon)
        latc = math.radians((cur_lat + goal_lat) / 2.0)
        x = dlon * math.cos(latc) * R
        y = dlat * R
        distance = math.hypot(x, y)
        bearing = math.atan2(x, y)  # radians: 0 -> north, + east
        return bearing, distance

    def plan(self, current_pose: Dict, goal: Dict, obstacles: List[Dict]) -> Dict:
        """
        Simple reactive planner:
        - Compute vector to goal (vx, vy)
        - For each obstacle, if within threat region, produce repulsive vector
        - Combine attractive + repulsive vectors -> velocity command
        """

        cur_lat = current_pose.get("lat")
        cur_lon = current_pose.get("lon")
        cur_alt = current_pose.get("alt", 0.0)
        yaw = current_pose.get("yaw", 0.0)

        goal_lat = goal.get("lat")
        goal_lon = goal.get("lon")
        goal_alt = goal.get("alt", cur_alt)

        # compute attraction vector in local meters
        bearing, dist = self._bearing_distance_to_goal(cur_lat, cur_lon, goal_lat, goal_lon)
        # desired speed magnitude
        desired_speed = min(self.max_speed, dist / 5.0)  # slow down near goal
        # convert bearing, speed -> vx (north), vy (east)
        vx = desired_speed * math.cos(bearing)
        vy = desired_speed * math.sin(bearing)
        vz = max(-2.0, min(2.0, (goal_alt - cur_alt)))  # simple altitude ramp (m/s)

        # apply repulsive forces from obstacles (in local frame where pos (0,0) is drone)
        rep_vx = 0.0
        rep_vy = 0.0
        for obs in obstacles:
            pos = obs.get("position", {})
            rx = float(pos.get("rel_x", 0.0))
            ry = float(pos.get("rel_y", 0.0))
            radius = float(obs.get("radius", 1.0))
            dist_obs = math.hypot(rx, ry)
            if dist_obs < self.min_clearance + radius + 0.5:
                # strong repulsion
                force = (1.0 / max(0.1, dist_obs)) * 5.0
                rep_vx += -force * (rx / (dist_obs + 1e-6))
                rep_vy += -force * (ry / (dist_obs + 1e-6))
            elif dist_obs < 10.0:
                # weaker repulsion
                force = (0.5 / max(0.1, dist_obs))
                rep_vx += -force * (rx / (dist_obs + 1e-6))
                rep_vy += -force * (ry / (dist_obs + 1e-6))

        # combine
        vx_total = vx + rep_vx
        vy_total = vy + rep_vy

        # cap speed
        speed = math.hypot(vx_total, vy_total)
        if speed > self.max_speed:
            scale = self.max_speed / speed
            vx_total *= scale
            vy_total *= scale

        # yaw_rate: simple pointing to velocity vector
        yaw_rate_cmd = 0.0
        if abs(vx_total) + abs(vy_total) > 1e-3:
            desired_bearing = math.degrees(math.atan2(vy_total, vx_total))
            yaw_error = desired_bearing - yaw
            # wrap
            while yaw_error > 180: yaw_error -= 360
            while yaw_error < -180: yaw_error += 360
            yaw_rate_cmd = max(-45.0, min(45.0, yaw_error * 0.5))  # deg/s

        return {"vx": vx_total, "vy": vy_total, "vz": vz, "yaw_rate": yaw_rate_cmd}
