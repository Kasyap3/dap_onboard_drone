# modules/flight/stabilization.py
"""
Stabilization controller (lightweight PID cascaded controller).

Responsibilities:
- Provide attitude control signals (roll, pitch, yaw rate, throttle) from target state & current state.
- Minimal, robust PID implementation appropriate for a companion controller or testing.

Key classes / functions:
- PID: simple PID regulator (stateful).
- StabilizationController:
    - update(target_state: dict, current_state: dict, dt: float) -> dict
    - Input shapes:
        target_state: {"roll": deg, "pitch": deg, "yaw": deg, "throttle": 0..1}
        current_state: {"roll": deg, "pitch": deg, "yaw": deg, "roll_rate": dps, ...}
    - Output:
        control: {"roll_cmd": -1..1, "pitch_cmd": -1..1, "yaw_rate_cmd": dps, "throttle_cmd": 0..1}
"""

from dataclasses import dataclass, field
import time
from typing import Dict


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    integrator: float = 0.0
    last_error: float = 0.0
    last_time: float = field(default_factory=time.time)
    integrator_limit: float = 1.0

    def reset(self):
        self.integrator = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

    def update(self, error: float, dt: float) -> float:
        """Compute PID output given error and timestep dt."""
        if dt <= 0:
            return 0.0
        self.integrator += error * dt
        # clamp integrator
        if self.integrator > self.integrator_limit:
            self.integrator = self.integrator_limit
        if self.integrator < -self.integrator_limit:
            self.integrator = -self.integrator_limit
        derivative = (error - self.last_error) / dt
        out = self.kp * error + self.ki * self.integrator + self.kd * derivative
        self.last_error = error
        return out


class StabilizationController:
    """
    Lightweight cascaded stabilization controller.

    Usage:
        ctrl = StabilizationController()
        cmd = ctrl.update(target_state, current_state, dt)

    Inputs:
      - target_state: {"roll": deg, "pitch": deg, "yaw": deg, "throttle": 0..1}
      - current_state: {"roll": deg, "pitch": deg, "yaw": deg, "roll_rate": dps, ...}
      - dt: seconds since last update

    Output:
      - dict with control commands:
        {"roll_cmd": -1..1, "pitch_cmd": -1..1, "yaw_rate_cmd": dps, "throttle_cmd": 0..1}
    """

    def __init__(self):
        # angle PIDs (outer loop)
        self.roll_pid = PID(kp=0.04, ki=0.002, kd=0.01, integrator_limit=10.0)
        self.pitch_pid = PID(kp=0.04, ki=0.002, kd=0.01, integrator_limit=10.0)
        # yaw rate controller (inner loop)
        self.yaw_pid = PID(kp=0.02, ki=0.0005, kd=0.001, integrator_limit=10.0)
        # throttle / altitude controller could be here (not implemented)
        self.last_time = time.time()

    def reset(self):
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()

    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def update(self, target_state: Dict, current_state: Dict, dt: float) -> Dict:
        """
        Compute control commands.

        dt: float seconds
        """

        # read states with defaults
        tgt_roll = float(target_state.get("roll", 0.0))
        tgt_pitch = float(target_state.get("pitch", 0.0))
        tgt_yaw = float(target_state.get("yaw", 0.0))
        tgt_throttle = float(target_state.get("throttle", 0.0))

        cur_roll = float(current_state.get("roll", 0.0))
        cur_pitch = float(current_state.get("pitch", 0.0))
        cur_yaw = float(current_state.get("yaw", 0.0))
        cur_yaw_rate = float(current_state.get("yaw_rate", 0.0))

        # angle errors (degrees)
        err_roll = tgt_roll - cur_roll
        err_pitch = tgt_pitch - cur_pitch
        # compute desired roll/pitch commands [-1..1] through PID
        roll_out = self.roll_pid.update(err_roll, dt)
        pitch_out = self.pitch_pid.update(err_pitch, dt)

        # yaw control: desire to achieve yaw (convert to yaw rate command)
        # simple P on yaw angle error to obtain yaw rate setpoint
        err_yaw = (tgt_yaw - cur_yaw)
        # wrap yaw error to [-180,180]
        while err_yaw > 180:
            err_yaw -= 360
        while err_yaw < -180:
            err_yaw += 360
        desired_yaw_rate = 2.0 * err_yaw  # deg/s per deg error (tunable)
        yaw_rate_error = desired_yaw_rate - cur_yaw_rate
        yaw_rate_cmd = self.yaw_pid.update(yaw_rate_error, dt)

        # throttle: clamp to [0,1]
        throttle_cmd = self._clamp(tgt_throttle, 0.0, 1.0)

        # map outputs to reasonable ranges
        # The roll/pitch outputs might be in degrees; normalize for actuation [-1,1]
        roll_cmd = self._clamp(roll_out / 20.0, -1.0, 1.0)
        pitch_cmd = self._clamp(pitch_out / 20.0, -1.0, 1.0)
        yaw_rate_cmd = max(-180.0, min(180.0, yaw_rate_cmd))

        return {
            "roll_cmd": roll_cmd,
            "pitch_cmd": pitch_cmd,
            "yaw_rate_cmd": yaw_rate_cmd,
            "throttle_cmd": throttle_cmd
        }
