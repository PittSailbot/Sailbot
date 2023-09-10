"""
Event blueprint class and common utility functions used in events
"""
from abc import abstractmethod

from rclpy.node import Node


class Event:
    """
    Basic blueprint for creating new events

    Attributes:
        - event_info (array) - provided starter information about the event
            - event_info = []

    Functions:
        - next_gps() - event logic which determines where to sail to next
    """

    REQUIRED_ARGS = 0

    def __init__(self, event_info):
        self._event_info = event_info
        self._node = Node(self.__class__.__name__)
        self.logging = self._node.get_logger()
        self.logging.info(f"Initializing {self.__class__.__name__}")

        if len(event_info) != self.REQUIRED_ARGS:
            raise TypeError(f"Expected {self.REQUIRED_ARGS} arguments, got {len(event_info)}")

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
