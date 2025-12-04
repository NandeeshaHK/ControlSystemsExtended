import math

def calculate_distance_global(lat1, lon1, lat2, lon2):
    """
    Calculate the ground distance between two global coordinates (lat/lon) in meters.
    """
    dLat = math.radians(lat2) - math.radians(lat1)
    dLon = math.radians(lon2) - math.radians(lon1)
    a = math.sin(0.5*dLat)**2 + math.sin(0.5*dLon)**2 * math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
    c = 2.0 * math.atan2(math.sqrt(abs(a)), math.sqrt(abs(1.0-a)))
    ground_dist = 6371 * 1000 * c
    return ground_dist

def new_lat_lon(lat, lon, hdg, dist, movement_head):
    """
    Calculate new latitude and longitude given current position, heading, distance, and movement heading.
    """
    lati = math.radians(lat)
    longi = math.radians(lon)
    rade = 6367489
    AD = dist / rade
    sumofangles = (hdg + movement_head + 360) % 360
    newheading = math.radians(sumofangles)
    newlati = math.asin(math.sin(lati) * math.cos(AD) + math.cos(lati) * math.sin(AD) * math.cos(newheading))
    newlongi = longi + math.atan2(math.sin(newheading) * math.sin(AD) * math.cos(lati), math.cos(AD) - math.sin(lati) * math.sin(newlati))
    ret_lat = int(round(math.degrees(newlati * 1e7)))
    ret_lon = int(round(math.degrees(newlongi * 1e7)))
    return ret_lat, ret_lon
