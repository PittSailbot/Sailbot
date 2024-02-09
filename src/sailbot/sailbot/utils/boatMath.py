"""
Math functions useful for sailbotting
"""
import math


def distance_between(waypoint1, waypoint2) -> float:
    """Calculates the distance between two GPS points using the Haversine formula
    # Args:
        - waypoint1 (eventUtils.Waypoint)
        - waypoint2 (eventUtils.Waypoint)
    # Returns:
        - distance in meters between points (float)
    """
    EARTH_RADIUS = 6371000

    # Convert latitude and longitude to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [waypoint1.lat, waypoint1.lon, waypoint2.lat, waypoint2.lon])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = EARTH_RADIUS * c

    return distance


# TODO: check
def angle_between(waypoint1, waypoint2) -> float:
    """Calculates the angle from waypoint1 to waypoint2 relative to north
    # Args:
        - waypoint1 (eventUtils.Waypoint)
        - waypoint2 (eventUtils.Waypoint)
    # Returns:
        - angle between points relative to north (float)
    """
    theta1 = math.radians(waypoint1.lat)
    theta2 = math.radians(waypoint2.lat)
    delta2 = math.radians(waypoint2.lon - waypoint1.lon)

    y = math.sin(delta2) * math.cos(theta2)
    x = math.cos(theta1) * math.sin(theta2) - math.sin(theta1) * math.cos(theta2) * math.cos(delta2)
    brng = math.atan(y / x)
    brng *= 180 / math.pi

    brng = (brng + 360) % 360

    return brng


def angleToPoint(heading, lat1, long1, lat2, long2):
    phi = math.atan2(long1 - long2, lat1 - lat2)

    return (360 - (math.degrees(phi) + heading + 180)) % 360


def convertDegMinToDecDeg(degMin):
    min = 0.0
    decDeg = 0.0

    min = math.fmod(degMin, 100.0)

    degMin = int(degMin / 100)
    decDeg = degMin + (min / 60)

    return decDeg


def remap(x, min1, max1, min2, max2):
    """Converts x from the range min1 <= x <= max1 to the proportional y from min2 <= y <= max2
    - Identical to arduino's map() function
    - ex: rescale(0.3, 0, 1, 0, 100) returns 30
    - ex: rescale(70, 0, 100, 0, 1) returns .7
    """
    x = min(max(x, min1), max1)
    return min2 + (max2 - min2) * ((x - min1) / (max1 - min1))
