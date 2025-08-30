import math

EARTH_RADIUS = 6371000.0  # meters

def integrate_latlon(lat, lon, vn, ve, dt):
    """
    Integrate latitude/longitude given velocities in the local tangent plane (north/east).
    lat, lon in radians
    vn, ve in m/s
    dt in seconds
    Returns updated (lat, lon) in radians
    """
    dlat = (vn / EARTH_RADIUS) * dt
    dlon = (ve / (EARTH_RADIUS * math.cos(lat))) * dt
    return lat + dlat, lon + dlon
