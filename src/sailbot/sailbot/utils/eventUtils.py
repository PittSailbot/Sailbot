"""
Event blueprint class and common utility functions used in events
"""
from abc import abstractmethod
import configparser
from rclpy.node import Node
from std_msgs.msg import String

from sailbot import constants as c
from sailbot.utils.utils import Waypoint

EVENT_DICT_INITIALIZED = False


class Event(Node):
    """Basic blueprint for creating new events"""

    def __init__(self):
        super().__init__(self.__class__.__name__)
        self.logging = self.get_logger()

        self.logging.info(f"Initializing {self.__class__.__name__}")

        self.target = None

        self.node_shutdown_sub = self.create_subscription(String, "event_shutdown", self.event_stop_callback, 10)
        self.set_event_sub = self.create_subscription(String, "set_event_target", self.ROS_setEventCallback, 10)

    def event_stop_callback(self, msg):
        raise KeyboardInterrupt()

    def ROS_setEventCallback(self, msg):
        self.target = Waypoint.from_msg(msg)

    @abstractmethod
    def event_loop(self):
        """Main event script logic. Executed continuously by ROS."""
        raise NotImplementedError


class EventFinished(Exception):
    """Signals that the event is finished and that it is safe to return to manual control"""
    pass
