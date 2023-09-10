"""
Event blueprint class and common utility functions used in events
"""
# Event descriptions can be found here: https://www.sailbot.org/wp-content/uploads/2022/05/SailBot-2022-Events.pdf

import configparser
import math
import os
import time
from abc import abstractmethod
from dataclasses import dataclass

from rclpy.node import Node

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if DOCKER == "True" else False

import sailbot.constants as c
from sailbot.utils import singleton

EVENT_DICT_INITIALIZED = False


def getEventDict():
    global EVENT_DICT_INITIALIZED, _EVENT_DICT
    if not EVENT_DICT_INITIALIZED:
        __generateEventDict()

    return _EVENT_DICT


def __generateEventDict():
    """
    waits to import the events to prevent circular dependencies
    """
    import sailbot.events.collisionAvoidance as collisionAvoidance
    import sailbot.events.endurance as endurance
    import sailbot.events.precisionNavigation as precisionNavigation
    import sailbot.events.search as search
    import sailbot.events.stationKeeping as stationKeeping

    eventDict = {
        c.config["MODES"]["MOD_RC"]: lambda *args: None,
        c.config["MODES"]["MOD_COLLISION_AVOID"]: collisionAvoidance.CollisionAvoidance,
        c.config["MODES"][
            "MOD_PRECISION_NAVIGATE"
        ]: precisionNavigation.Precision_Navigation,
        c.config["MODES"]["MOD_ENDURANCE"]: endurance.Endurance,
        c.config["MODES"]["MOD_STATION_KEEPING"]: stationKeeping.Station_Keeping,
        c.config["MODES"]["MOD_SEARCH"]: search.Search,
    }
    global EVENT_DICT_INITIALIZED, _EVENT_DICT
    EVENT_DICT_INITIALIZED = True
    _EVENT_DICT = eventDict


class EventDefaults:
    """
    defaults are stored as a class variable so that they are only initialized once
    """

    if os.path.isfile("eventDefaults.ini"):
        prefix = ""
    elif os.path.isfile("/workspace/eventDefaults.ini"):
        prefix = "/workspace/"
    elif os.path.isfile("/home/pi/ros2_ws/eventDefaults.ini"):
        prefix = "/home/pi/ros2_ws/"
    else:
        raise Exception(f"cannot find eventDefaults.ini file in {os.getcwd()}")

    defaults = configparser.ConfigParser()
    defaults.read(f"{prefix}eventDefaults.ini")


@dataclass()
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
        self.lon += (
            (dx / EARTH_RADIUS) * (180 / math.pi) / math.cos(self.lat * math.pi / 180)
        )


class Event:
    """
    Basic blueprint for creating new events

    Attributes:
        - event_info (array) - provided starter information about the event
            - event_info = []

    Functions:
        - next_gps() - event logic which determines where to sail to next
    """

    base_required_args = ["gps"]

    def __init__(self, event_info):
        self._event_info = event_info
        self._node = Node(self.__class__.__name__)
        self.logging = self._node.get_logger()
        self.logging.info(f"Initializing {self.__class__.__name__}")

        for key in self.required_args:
            if key not in event_info:
                raise KeyError(
                    f"{self.__class__.__name__} expects dict with key: {key}, keys found: {event_info.keys()}"
                )

        for key in self.base_required_args:
            if key not in event_info:
                raise KeyError(
                    f"Events expect dict with key: {key}, keys found: {event_info.keys()}"
                )

        for key in event_info.keys():
            if key not in self.required_args and key not in self.base_required_args:
                self.logging.warning(
                    f"Found unused key in {self.__class__.__name__}: {key}"
                )

        self.gps = event_info["gps"]

    @abstractmethod
    def next_gps(self):
        """
        Main event script logic. Executed continuously by boatMain.

        Returns either:
            - The next GPS point that the boat should sail to stored as a Waypoint object
            - OR None to signal the boat to drop sails and clear waypoint queue
            - OR EventFinished exception to signal that the event has been completed
        """

        raise NotImplementedError

    def has_reached_waypoint(
        self,
        waypoint,
        distance=float(c.config["CONSTANTS"]["reached_waypoint_distance"]),
    ):
        """Returns true/false if the boat is close enough to the waypoint"""
        return distance_between(self.gps.waypoint, waypoint) < distance


class EventFinished(Exception):
    """Signals that the event is finished and that it is safe to return to manual control"""

    pass


"""
def PID():  #hana
    total_error = 0.0
    oldError = 0.0
    oldTime = time.time()
    last_pnt_x, .last_pnt_y = None,None
    gps_class = gps()
    # New PID stuff
    if (time.time() - oldTime < 100):  # Only runs every tenth of a second #new
        # Finds the angle the boat should take
        error = boat_RefObj.targetAngle - compass.angle  # Finds how far off the boat is from its goal
        totalError += error  # Gets the total error to be used for the integral gain
        derivativeError = (error - oldError) / (time.time() - oldtime)  # Gets the change in error for the derivative portion
        deltaAngle = c.config['CONSTANTS']["P"] * error + c.config['CONSTANTS']["I"] * totalError + c.config['CONSTANTS']["D"] * derivativeError  # Finds the angle the boat should be going

        # Translates the angle into lat and log so goToGPS won't ignore it
        boat_RefObj.currentAngle = getCoordinateADistanceAlongAngle(1000, deltaAngle + self.boat_RefObj.compass.angle)

        # Resets the variable
        oldTime = time.time()
        oldError = error
"""


def SK_f(self, x, a1, b1, a2, b2):
    return self.SK_m(a1, b1, a2, b2) * x + self.SK_v(a1, b1, a2, b2)  # f(x)=mx+b


def SK_m(self, a1, b1, a2, b2):
    return (b2 - b1) / (a2 - a1)  # m: slope between two lines


def SK_v(self, a1, b1, a2, b2):
    return b1 - (self.SK_m(a1, b1, a2, b2) * a1)  # b: +y between two lines


def SK_I(self, M1, V1, M2, V2):
    return (V2 - V1) / (M1 - M2)  # find x-cord intersect between two lines


def SK_d(self, a1, b1, a2, b2):
    return math.sqrt(
        (a2 - a1) ** 2 + (b2 - b1) ** 2
    )  # find distance between two points


def distance_between(waypoint1, waypoint2):
    """Calculates the distance between two GPS points using the Haversine formula
    # Args:
        - waypoint1 (eventUtils.Waypoint)
        - waypoint2 (eventUtils.Waypoint)
    # Returns:
        - distance in meters between points (float)
    """
    EARTH_RADIUS = 6371000

    # Convert latitude and longitude to radians
    lat1, lon1, lat2, lon2 = map(
        math.radians, [waypoint1.lat, waypoint1.lon, waypoint2.lat, waypoint2.lon]
    )

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = EARTH_RADIUS * c

    return distance


def getDefaultEventParams(eventEnum):
    """
    generated a event_info dict using the default values found in eventDefaults.ini
    """
    key = None
    for testKey, testValue in c.config["MODES"].items():
        if testValue == eventEnum:
            key = testKey
            break

    if key is None:
        raise KeyError(f"{eventEnum} was not found in config['MODES']")

    defaults = EventDefaults.defaults[key.upper()]

    event_info = {}
    for key, value in defaults.items():
        if str(value).lower().startswith("waypoint"):
            args = value.split(" ")[1:]
            event_info[key] = Waypoint(float(args[0]), float(args[1]))

        elif str(value).lower().startswith("float"):
            floatVal = value.split(" ")[1]
            event_info[key] = float(floatVal)

        else:
            event_info[key] = value

    return event_info
