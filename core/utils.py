# core/utils.py
"""
Utility helpers: geo transforms, clamping, quaternion/euler, timing utilities.
"""

import math
from typing import Tuple

# Earth constants for haversine etc.
EARTH_RADIUS_M = 6371000.0


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Approximate great-circle distance between two lat/lon points in meters.
    """
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2.0)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2.0)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS_M * c


def gps_to_local_meters(origin_lat: float, origin_lon: float, lat: float, lon: float) -> Tuple[float, float]:
    """
    Very simple equirectangular projection to convert lat/lon to local meters relative to origin.
    Suitable for short distances (< few km).
    Returns (dx_m, dy_m) where dx is east, dy is north.
    """
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    o_lat = math.radians(origin_lat)
    o_lon = math.radians(origin_lon)
    dlat = lat_rad - o_lat
    dlon = lon_rad - o_lon
    dx = dlon * math.cos((lat_rad + o_lat) / 2.0) * EARTH_RADIUS_M
    dy = dlat * EARTH_RADIUS_M
    return dx, dy


def local_meters_to_gps(origin_lat: float, origin_lon: float, dx: float, dy: float) -> Tuple[float, float]:
    """
    Inverse of gps_to_local_meters; returns (lat, lon).
    """
    dlat = dy / EARTH_RADIUS_M
    dlon = dx / (EARTH_RADIUS_M * math.cos(math.radians(origin_lat)))
    lat = origin_lat + math.degrees(dlat)
    lon = origin_lon + math.degrees(dlon)
    return lat, lon


def clamp(x: float, a: float, b: float) -> float:
    return max(a, min(b, x))


def wrap_angle_deg(angle: float) -> float:
    """Wrap angle to [-180, 180] degrees"""
    a = angle % 360.0
    if a > 180.0:
        a -= 360.0
    return a
