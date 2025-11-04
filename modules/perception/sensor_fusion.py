# modules/perception/sensor_fusion.py
"""
Sensor fusion utilities for perception.

- Simple fusion that combines camera-based detections and LiDAR point estimates to produce
  unified obstacle list with estimated position and radius.

API:
- fuse_detections(camera_dets: List[Dict], lidar_points: Optional[np.ndarray]) -> List[Dict]
"""

from typing import List, Dict, Optional
import numpy as np


def fuse_detections(camera_dets: List[Dict], lidar_points: Optional[np.ndarray] = None) -> List[Dict]:
    """
    Combine camera detections and lidar pointcloud to yield obstacles with conservative radius.

    camera_dets: list with bbox and rel position
      e.g. {"label","confidence","bbox":[x,y,w,h],"position":{"rel_x":..., "rel_y":...}}
    lidar_points: Nx3 array in vehicle frame or None

    Returns:
      list of fused obstacles: {"label","confidence","position":{"rel_x","rel_y"}, "radius": meters}
    """

    fused = []
    # Basic pass-through / merge: if lidar_points present, cluster near camera detection positions
    for det in camera_dets:
        pos = det.get("position", {"rel_x": None, "rel_y": None})
        rel_x = float(pos.get("rel_x", 0.0)) if pos.get("rel_x") is not None else 0.0
        rel_y = float(pos.get("rel_y", 0.0)) if pos.get("rel_y") is not None else 0.0
        radius = 1.0  # default 1m radius
        conf = det.get("confidence", 0.5)
        label = det.get("label", "object")

        # if lidar exists, estimate radius from nearest points
        if lidar_points is not None and lidar_points.size > 0:
            # find points within +/- 2m box around estimated position
            dx = lidar_points[:, 0] - rel_x
            dy = lidar_points[:, 1] - rel_y
            d2 = dx * dx + dy * dy
            close = lidar_points[d2 < 4.0]  # within 2m
            if close.shape[0] > 0:
                # radius ~ std dev of points or max dist
                distances = np.sqrt(np.sum((close[:, :2] - np.array([rel_x, rel_y])) ** 2, axis=1))
                radius = float(max(0.5, np.mean(distances) + np.std(distances)))
                conf = min(1.0, conf + 0.2)

        fused.append({
            "label": label,
            "confidence": float(conf),
            "position": {"rel_x": rel_x, "rel_y": rel_y},
            "radius": float(radius)
        })

    # If lidar had clusters not covered by camera, optionally add them (not implemented)
    return fused
