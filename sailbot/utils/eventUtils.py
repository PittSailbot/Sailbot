"""
Event blueprint class and common utility functions used in events
"""
from abc import abstractmethod
import configparser
from rclpy.node import Node

import sailbot.constants as c
from sailbot.utils.utils import Waypoint

EVENT_DICT_INITIALIZED = False


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
    required_args = {}

    def __init__(self, event_info):
        self._event_info = event_info
        self._node = Node(self.__class__.__name__)
        self.logging = self._node.get_logger()
        self.logging.info(f"Initializing {self.__class__.__name__}")

        for key in self.required_args:
            if key not in event_info:
                raise KeyError(f"{self.__class__.__name__} expects dict with key: {key}, keys found: {event_info.keys()}")

        for key in self.base_required_args:
            if key not in event_info:
                raise KeyError(f"Events expect dict with key: {key}, keys found: {event_info.keys()}")

        for key in event_info.keys():
            if key not in self.required_args and key not in self.base_required_args:
                self.logging.warning(f"Found unused key in {self.__class__.__name__}: {key}")

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


class EventFinished(Exception):
    """Signals that the event is finished and that it is safe to return to manual control"""

    pass


def getEventDict():
    global EVENT_DICT_INITIALIZED, _EVENT_DICT
    if not EVENT_DICT_INITIALIZED:
        __generateEventDict()

    return _EVENT_DICT


def __generateEventDict():
    """
    waits to import the events to prevent circular dependencies
    """
    import sailbot.events.endurance as endurance
    import sailbot.events.precisionNavigation as precisionNavigation
    import sailbot.events.search as search
    import sailbot.events.stationKeeping as stationKeeping

    eventDict = {
        c.config["MODES"]["MOD_RC"]: lambda *args: None,
        c.config["MODES"][
            "MOD_PRECISION_NAVIGATE"
        ]: precisionNavigation.PrecisionNavigation,
        c.config["MODES"]["MOD_ENDURANCE"]: endurance.Endurance,
        c.config["MODES"]["MOD_STATION_KEEPING"]: stationKeeping.StationKeeping,
        c.config["MODES"]["MOD_SEARCH"]: search.Search,
    }
    global EVENT_DICT_INITIALIZED, _EVENT_DICT
    EVENT_DICT_INITIALIZED = True
    _EVENT_DICT = eventDict


class EventDefaults:
    """
    defaults are stored as a class variable so that they are only initialized once
    """
    defaults = configparser.ConfigParser()
    defaults.read(f"{c.root_dir}/eventDefaults.ini")


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
