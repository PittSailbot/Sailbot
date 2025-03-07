"""
Event blueprint class and common utility functions used in events
"""

import configparser
from abc import abstractmethod

from rclpy.node import Node
from std_msgs.msg import Int32, String

from sailbot import constants as c
from sailbot.utils.utils import ControlState, Waypoint

EVENT_DICT_INITIALIZED = False


class Event(Node):
    """
    Basic blueprint for creating new events

    Attributes:
        - event_info (array) - provided starter information about the event
            - event_info = []

    Functions:
        - next_gps() - event logic which determines where to sail to next
    """

    base_required_args = []
    required_args = {}

    def __init__(self, event_info):
        self._event_info = event_info
        super().__init__(self.__class__.__name__)
        self.logging = self.get_logger()

        self.event_control_state = self.create_publisher(Int32, "/boat/event_control_state", 1)
        timer_period = 1.0  # seconds
        self.control_state_timer = self.create_timer(timer_period, self.control_state_timer_callback)

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

        self.node_shutdown_sub = self.create_subscription(String, "event_shutdown", self.event_stop_callback, 10)
        self.pub = self.create_publisher(String, "next_gps", 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def event_stop_callback(self, msg):
        raise KeyboardInterrupt()

    def timer_callback(self):
        try:
            waypoint = self.next_gps()
            target = waypoint.to_msg()
        except Exception as e:
            self.logging.error(f"{self.__class__.__name__} failed to get next GPS with error: {e}")
            target = String()
            target.data = ""

        self.pub.publish(target)
        self.logging.debug(f"{self.__class__.__name__} publishing next_GPS: {target.data}")

    def control_state_timer_callback(self):
        self.event_control_state.publish(Int32(data=ControlState.AUTO))

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
        c.config["MODES"]["MOD_PRECISION_NAVIGATE"]: precisionNavigation.PrecisionNavigation,
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


def get_default_event_params(event_enum):
    """
    generated a event_info dict using the default values found in eventDefaults.ini
    """
    key = None
    for test_key, test_value in c.config["MODES"].items():
        if test_value == event_enum:
            key = test_key
            break

    if key is None:
        raise KeyError(f"{event_enum} was not found in config['MODES']")

    defaults = EventDefaults.defaults[key.upper()]

    event_info = {}
    for key, value in defaults.items():
        if str(value).lower().startswith("waypoint"):
            args = value.split(" ")[1:]
            event_info[key] = Waypoint(float(args[0]), float(args[1]))

        elif str(value).lower().startswith("float"):
            float_val = value.split(" ")[1]
            event_info[key] = float(float_val)

        else:
            event_info[key] = value

    return event_info
