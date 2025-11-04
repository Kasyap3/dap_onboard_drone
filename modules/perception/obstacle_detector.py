# modules/perception/obstacle_detector.py
"""
Obstacle detector interface.

- Minimal, model-agnostic wrapper for running onboard perception.
- Returns list of obstacles as dictionaries with position (relative or global),
  footprint radius, and confidence.

API:
- ObstacleDetector(model_path: Optional[str]=None)
- detect(frame: np.ndarray) -> List[Dict]

Input:
  - frame: HxWxC numpy image (uint8 or float32)

Output:
  - List of obstacles: [{"label":"tree","confidence":0.92,"bbox":[x,y,w,h], "position": {"rel_x":..., "rel_y":...}}]
"""

from typing import List, Dict, Optional
import numpy as np
import time
import os


class ObstacleDetector:
    def __init__(self, model_path: Optional[str] = None):
        self.model_path = model_path
        self.loaded = False
        self.model = None
        if model_path:
            self.load_model(model_path)

    def load_model(self, model_path: str):
        """
        Load model. This is a placeholder that simulates loading.
        Replace with actual torch/onnx runtime code as needed.
        """
        if not os.path.exists(model_path):
            # Do not raise here; allow runtime graceful degradation
            print(f"[ObstacleDetector] model not found at {model_path}, using stub detector.")
            self.loaded = False
            self.model = None
            return
        # Example: load with torch or onnxruntime; omitted here
        self.model = {"path": model_path}
        self.loaded = True
        print(f"[ObstacleDetector] loaded model from {model_path}")

    def detect(self, frame: np.ndarray) -> List[Dict]:
        """
        Perform detection on a single frame.

        Simple default stub: returns one fake obstacle if frame mean intensity < threshold.
        Replace with actual inference call as required.

        Input:
          - frame: np.ndarray (H,W,C)

        Output:
          - List[Dict] obstacles
        """
        # Basic validation
        if frame is None:
            return []

        # stub logic: if image dark, return an obstacle
        avg = float(np.mean(frame))
        obstacles = []
        timestamp = time.time()
        if not self.loaded:
            # very cheap, approximate heuristic-based detection
            if avg < 100:
                # place a fake bbox in center
                h, w = frame.shape[:2]
                bbox = [int(w * 0.4), int(h * 0.4), int(w * 0.2), int(h * 0.2)]
                obstacles.append({
                    "label": "unknown",
                    "confidence": 0.5,
                    "bbox": bbox,
                    "timestamp": timestamp,
                    "position": {"rel_x": 5.0, "rel_y": 0.0}  # meters ahead
                })
        else:
            # If a model were loaded, we'd perform inference here.
            # Simulate model latency and a detection
            time.sleep(0.01)
            h, w = frame.shape[:2]
            bbox = [int(w * 0.3), int(h * 0.3), int(w * 0.4), int(h * 0.4)]
            obstacles.append({
                "label": "object",
                "confidence": 0.92,
                "bbox": bbox,
                "timestamp": timestamp,
                "position": {"rel_x": 4.2, "rel_y": -1.2}
            })

        return obstacles
