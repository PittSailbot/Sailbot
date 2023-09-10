"""Very commonly used utility functions and classes"""
import math
import os
from dataclasses import dataclass

from sailbot import constants as c
from sailbot.peripherals.GPS import GPS
from sailbot.utils.boatMath import distance_between

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if DOCKER == "True" else False

if not DOCKER:
    pass
else:
    pass


def singleton(cls):
    """A decorator which prevents duplicate classes from being created.
    Useful for physical objects where only one exists.
        - Import, then invoke use @singleton before class definition"""
    instances = {}

    def get_instance(*args, **kwargs):
        if cls not in instances:
            instances[cls] = cls(*args, **kwargs)
        return instances[cls]

    return get_instance


@dataclass(slots=True)
class Waypoint:
    """
    A GPS marker for buoys, travel destinations, etc.
        - Initialize using Waypoint(latitude, longitude)
    """

    lat: float
    lon: float

    def __str__(self):
        return f"({self.lat}, {self.lon})"

    def __repr__(self):
        return f"Waypoint({self.lat, self.lon})"

    def add_meters(self, dx, dy):
        """Updates the waypoint gps by adding meters to the latitude and longitude"""
        EARTH_RADIUS = 6371000

        self.lat += (dy / EARTH_RADIUS) * (180 / math.pi)
        self.lon += (dx / EARTH_RADIUS) * (180 / math.pi) / math.cos(self.lat * math.pi / 180)


def has_reached_waypoint(waypoint, distance=float(c.config["CONSTANTS"]["reached_waypoint_distance"])):
    """Returns true/false if the boat is close enough to the waypoint"""
    a = GPS()
    boat_gps = Waypoint(a.latitude, a.longitude)
    return distance_between(boat_gps, waypoint) < distance


class dummyObject:
    """
    a class that can be used as a placeholder for something else, usually a physical sensor that is not present.
    allows for setting of arbitrary variables that can be read and written to
    """

    def __init__(self, *args, **kwargs):
        pass
