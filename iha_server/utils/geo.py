from math import radians, cos, sin, asin, sqrt

def haversine_m(lat1, lon1, lat2, lon2):
    """Distance in meters."""
    R = 6371000.0
    dlat = radians(lat2-lat1)
    dlon = radians(lon2-lon1)
    a = sin(dlat/2)**2 + cos(radians(lat1))*cos(radians(lat2))*sin(dlon/2)**2
    c = 2*asin(sqrt(a))
    return R*c