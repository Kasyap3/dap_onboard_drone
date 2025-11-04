"""
config.py
Central configuration for Drone_Onboard_AV.
Contains communication settings, model paths, and flight thresholds.
"""

from pathlib import Path

# --- Base Paths ---
BASE_DIR = Path(__file__).resolve().parent
MODEL_DIR = BASE_DIR / "models"
LOG_DIR = BASE_DIR / "data" / "logs"

# --- Communication Settings ---
COMMS = {
    "mavlink_connection": "/dev/ttyAMA0",  # or UDP: 14550
    "baudrate": 57600,
    "ground_station_ip": "192.168.10.2",
    "ground_station_port": 5000,
}

# --- Sensor Settings ---
SENSORS = {
    "camera": True,
    "lidar": False,
    "gps": True,
    "imu": True,
}

# --- Flight Limits ---
FLIGHT_LIMITS = {
    "max_altitude": 120.0,  # meters
    "min_battery": 20.0,    # percentage
    "failsafe_timeout": 5.0 # seconds without telemetry
}

# --- Model Settings ---
MODELS = {
    "object_detection": str(MODEL_DIR / "objectdetection_yolo_v1.pt"),
    "obstacle_avoidance": str(MODEL_DIR / "obstacleavoidance_unet_v2.onnx"),
}

# --- Logging ---
LOGGING = {
    "level": "INFO",
    "log_to_file": True,
    "filename": str(LOG_DIR / "onboard_av.log"),
}
