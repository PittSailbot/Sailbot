"""Very commonly used utility functions and classes"""
import math
from dataclasses import dataclass
from datetime import datetime
import json

import rclpy
from rclpy.executors import ShutdownException, TimeoutException

from sailbot import constants as c
from sailbot.utils.boatMath import distance_between


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

    def toJson(self):
        return json.dumps({"lat": self.lat, "lon": self.lon})
    
    def fromJson(json_data):
        if str(json_data).upper() == 'NONE':
            return None
        
        data = json.loads(json_data)
        return Waypoint(data['lat'], data['lon'])


class DummyObject:
    """
    a class that can be used as a placeholder for something else, usually a physical sensor that is not present.
    allows for setting of arbitrary variables that can be read and written to
    """

    def __init__(self, *args, **kwargs):
        pass


# TODO: make function use ROS to resolve circulat import error
def has_reached_waypoint(waypoint, distance=float(c.config["CONSTANTS"]["reached_waypoint_distance"])):
    """Returns true/false if the boat is close enough to the waypoint"""
    # a = GPS()
    # boat_gps = Waypoint(a.latitude, a.longitude)
    # return distance_between(boat_gps, waypoint) < distance
    return None


def ros_spin_some(node, executor=None, timeout_sec=0, wait_condition=lambda: False):
    """
    execute ros callbacks until there are no more available or for timeout_sec, whichever comes first
    if timeout_sec is 0 then it will execute callbacks until there are no more available
    """
    if timeout_sec != 0:
        endTime = datetime.now() + timeout_sec
    executor = rclpy.get_global_executor() if executor is None else executor
    executor.add_node(node)
    while True:
        if timeout_sec != 0:
            remainingTime = endTime - datetime.now()
            if remainingTime <= 0:
                break
        else:
            remainingTime = 0.0

        try:
            handler, _, _ = executor.wait_for_ready_callbacks(
                remainingTime, None, wait_condition
            )
        except ShutdownException:
            pass
        except TimeoutException:
            break
        else:
            handler()
            if handler.exception() is not None:
                raise handler.exception()

    executor.remove_node(node)
