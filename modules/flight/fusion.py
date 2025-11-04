# modules/flight/fusion.py
"""
IMU + GPS + Barometer sensor fusion.

Provides a simple complementary filter / lightweight EKF stub to estimate pose (x,y,z,roll,pitch,yaw).

Responsibilities:
- Provide state estimate from raw sensors.
- Keep API minimal: update(imu, gps, baro, dt) -> dict pose

Inputs:
  - imu: dict {"ax","ay","az","gx","gy","gz"} (m/s^2, deg/s)
  - gps: dict {"lat","lon","alt"} or None
  - baro: dict {"pressure":..., "alt": ...} or None
  - dt: seconds

Output:
  - pose dict: {"x","y","z","roll","pitch","yaw","vx","vy","vz"}
"""

import math
import time
from typing import Dict, Optional


class AttitudeEstimator:
    """
    Lightweight estimator. Not a full EKF — designed for companion-level estimation.
    """

    def __init__(self):
        # internal state (NED-like)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        # attitude in degrees
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = time.time()

        # complementary constants
        self.alpha_att = 0.98  # trust gyro integration for short term
        self.alpha_pos = 0.9   # trust GPS slowly

    def reset(self):
        self.__init__()

    def update(self, imu: Dict, gps: Optional[Dict], baro: Optional[Dict], dt: float) -> Dict:
        """
        imu: {"ax","ay","az","gx","gy","gz"} axial units m/s^2 and deg/s
        gps: {"lat","lon","alt"} or None
        baro: {"alt": meters} or None
        dt: float seconds
        """
        if dt <= 0:
            return self.get_state()

        # Integrate gyro for attitude
        gx = imu.get("gx", 0.0)
        gy = imu.get("gy", 0.0)
        gz = imu.get("gz", 0.0)
        # assume small angles, degrees
        self.roll += gx * dt
        self.pitch += gy * dt
        self.yaw += gz * dt

        # Accelerometer for roll/pitch correction (approx)
        ax = imu.get("ax", 0.0)
        ay = imu.get("ay", 0.0)
        az = imu.get("az", 9.81)
        # compute estimated roll/pitch from accelerometer
        # avoid zero division
        try:
            roll_acc = math.degrees(math.atan2(ay, az))
            pitch_acc = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
        except Exception:
            roll_acc = self.roll
            pitch_acc = self.pitch

        # complementary filter
        self.roll = self.alpha_att * self.roll + (1 - self.alpha_att) * roll_acc
        self.pitch = self.alpha_att * self.pitch + (1 - self.alpha_att) * pitch_acc

        # Integrate acceleration to velocity (remove gravity by rotating accel vector -> rough)
        # This is an approximation: subtract gravity on z only
        # Convert to body frame approx -> not implementing full rotation here
        self.vx += imu.get("ax", 0.0) * dt
        self.vy += imu.get("ay", 0.0) * dt
        self.vz += (imu.get("az", 0.0) - 9.81) * dt

        # Integrate to position
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt

        # If GPS present, slowly correct position (simple complementary)
        if gps is not None:
            # GPS -> convert lat/lon to local meters roughly (equirectangular approx)
            lat = float(gps.get("lat"))
            lon = float(gps.get("lon"))
            alt = float(gps.get("alt", self.z))
            # We don't have origin here; user/system must offset externally.
            # For simplicity, we'll only correct z from alt if provided
            self.z = self.alpha_pos * self.z + (1 - self.alpha_pos) * alt

        # If barometer present, fuse altitude more strongly
        if baro and "alt" in baro:
            baro_alt = float(baro.get("alt"))
            self.z = 0.7 * self.z + 0.3 * baro_alt

        return self.get_state()

    def get_state(self) -> Dict:
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "vx": self.vx,
            "vy": self.vy,
            "vz": self.vz,
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw
        }
