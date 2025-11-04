# core/model_manager.py
"""
ModelManager: lightweight runtime manager for onboard ML models.

Responsibilities:
- Load models from models_repository path (supports .pt, .onnx, .tflite placeholders).
- Cache loaded models in memory.
- Provide run_inference(model_name, input_data) -> output
- Gracefully degrade if frameworks are absent.

Notes:
- This is intentionally framework-agnostic. For production, add specific runners:
  - PyTorch: torch.load / model.eval()
  - ONNX: onnxruntime.InferenceSession
  - TensorRT: tensorrt engine
"""

import importlib
import logging
import os
import time
from typing import Any, Dict, Optional

logger = logging.getLogger("core.model_manager")
logger.setLevel(logging.INFO)
if not logger.handlers:
    ch = logging.StreamHandler()
    ch.setFormatter(logging.Formatter("[%(asctime)s] %(levelname)s %(message)s"))
    logger.addHandler(ch)

# Try optional imports (if available)
try:
    import torch
except Exception:
    torch = None

try:
    import onnxruntime
except Exception:
    onnxruntime = None

class ModelManager:
    def __init__(self, model_root: str = "models_repository"):
        self.model_root = model_root
        self._cache: Dict[str, Any] = {}  # model_name -> loaded model object

    def _model_path(self, model_name: str) -> Optional[str]:
        # model_name expected like 'obstacle_detection_yolo_v1.pt'
        p = os.path.join(self.model_root, model_name)
        if os.path.exists(p):
            return p
        # also support nested folders: model_name may be 'perception/obstacle_...'
        nested = os.path.join(self.model_root, model_name.replace("/", os.sep))
        return nested if os.path.exists(nested) else None

    def load_model(self, model_name: str) -> Optional[Any]:
        """
        Load model by filename. Returns loaded object or None.
        """
        if model_name in self._cache:
            return self._cache[model_name]

        p = self._model_path(model_name)
        if p is None:
            logger.warning("ModelManager: model %s not found under %s", model_name, self.model_root)
            return None

        ext = os.path.splitext(p)[1].lower()
        model = None
        start = time.time()
        try:
            if ext in (".pt", ".pth") and torch is not None:
                model = torch.jit.load(p) if p.endswith(".pt") else torch.load(p, map_location="cpu")
                model.eval()
                logger.info("Loaded PyTorch model %s (%.2fs)", model_name, time.time() - start)
            elif ext == ".onnx" and onnxruntime is not None:
                sess = onnxruntime.InferenceSession(p, providers=["CPUExecutionProvider"])
                model = sess
                logger.info("Loaded ONNX model %s (%.2fs)", model_name, time.time() - start)
            else:
                # fallback: store path as marker; user must implement runner
                model = {"path": p, "ext": ext}
                logger.info("Registered model path for %s", model_name)
        except Exception:
            logger.exception("Failed to load model %s", model_name)
            model = None

        if model is not None:
            self._cache[model_name] = model
        return model

    def unload_model(self, model_name: str):
        if model_name in self._cache:
            del self._cache[model_name]
            logger.info("Unloaded model %s", model_name)

    async def run_inference(self, model_name: str, input_data: Any, timeout_s: float = 1.0) -> Any:
        """
        Run model inference. The actual runner depends on loaded model type.
        - For torch: call model(input_tensor)
        - For onnxruntime: sess.run(...)
        - For fallback: return None

        NOTE: This is a CPU-friendly wrapper. For GPU acceleration, integrate GPU runners.
        """
        model = self._cache.get(model_name) or self.load_model(model_name)
        if model is None:
            return None

        start = time.time()
        # PyTorch
        if torch is not None and hasattr(model, "eval"):
            try:
                input_tensor = input_data  # expect correct preprocessed tensor
                with torch.no_grad():
                    out = model(input_tensor)
                logger.debug("Inference (torch) took %.3fs", time.time() - start)
                return out
            except Exception:
                logger.exception("PyTorch inference failed")
                return None

        # ONNX
        if onnxruntime is not None and hasattr(model, "run"):
            try:
                sess = model
                input_name = sess.get_inputs()[0].name
                feed = {input_name: input_data}
                out = sess.run(None, feed)
                logger.debug("Inference (onnx) took %.3fs", time.time() - start)
                return out
            except Exception:
                logger.exception("ONNX inference failed")
                return None

        # fallback: placeholder runner
        logger.debug("Fallback model runner for %s (no framework)", model_name)
        await asyncio.sleep(0.01)  # simulate small latency
        return {"status": "ok", "model": model_name}
