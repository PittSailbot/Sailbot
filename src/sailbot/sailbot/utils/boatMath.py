"""
Math functions useful for sailbotting
"""

import math

import numpy as np

from sailbot import constants as c


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


def angle_to_point(lat1, lon1, lat2, lon2):
    """
    Calculate the compass angle (bearing) between two GPS coordinates.

    Args:
    lat1 (float): Latitude of the first point in degrees.
    lon1 (float): Longitude of the first point in degrees.
    lat2 (float): Latitude of the second point in degrees.
    lon2 (float): Longitude of the second point in degrees.

    Returns:
    float: Compass angle in degrees (0 to 360), relative to the north direction.
    """
    # Convert degrees to radians
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    # Calculate the differences in longitudes and latitudes
    delta_lon = lon2 - lon1
    y = math.sin(delta_lon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)

    # Calculate the compass angle (bearing)
    angle = math.atan2(y, x)
    angle = math.degrees(angle)
    angle = (angle + 360) % 360  # Normalize angle to be between 0 and 360 degrees

    return angle


def remap(x, min1, max1, min2, max2):
    """Converts x from the range min1 <= x <= max1 to the proportional y from min2 <= y <= max2
    - Identical to arduino's map() function
    - ex: rescale(0.3, 0, 1, 0, 100) returns 30
    - ex: rescale(70, 0, 100, 0, 1) returns .7
    """
    x = min(max(x, min1), max1)
    return min2 + (max2 - min2) * ((x - min1) / (max1 - min1))


def get_no_go_zone_bounds(wind_angle, compass_angle):
    wind_angle += compass_angle
    no_go_angle = float(c.config["NAVIGATION"]["no_go_angle"])
    no_go_zone_left_bound = (wind_angle - no_go_angle / 2) % 360
    no_go_zone_right_bound = (wind_angle + no_go_angle / 2) % 360

    return no_go_zone_left_bound, no_go_zone_right_bound


def calculateCoordinates(x0, y0, angle_in_degrees, distance_in_meters):
    earth_radius = 6371000

    angular_distance = distance_in_meters / earth_radius
    angle_in_radians = math.radians(angle_in_degrees)

    new_x = x0 + math.cos(angle_in_radians) * angular_distance
    new_y = y0 + math.sin(angle_in_radians) * angular_distance

    return new_x, new_y


def is_within_angle(b, a, c) -> bool:
    """Checks if the angle b, is contained within angle AC. Used to check if boat is pointed within no-go-zone"""
    if a > c:
        b = b % 360

        # Check if the heading is within the wrapped bounds
        if b >= a or b <= c:
            return True
    else:
        # Bounds don't wrap around
        if a <= b <= c:
            return True

    return False


def degrees_between(angle1, angle2) -> float:
    """
    Computes the number of degrees between two angles measured in degrees.

    Parameters:
        angle1 (float): The first angle in degrees.
        angle2 (float): The second angle in degrees.

    Returns:
        float: The number of degrees between the two angles.
    """
    angle1 = angle1 % 360
    angle2 = angle2 % 360

    diff = abs(angle1 - angle2)
    return min(diff, 360 - diff)


def quaternion_to_euler(x, y, z, w) -> (float, float, float):
    """Convert a quaternion oreintation into euler angles (roll, pitch, yaw)

    Returns:
        tuple(float, float, float): roll, pitch and yaw in degrees (counterclockwise)
    """

    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    roll = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    yaw = math.degrees(math.atan2(t3, t4))

    # Radians -> Degrees
    yaw = yaw * (180 / math.pi)
    pitch = pitch * (180 / math.pi)
    roll = roll * (180 / math.pi)

    return roll, pitch, yaw
