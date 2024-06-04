import importlib
import math
import os
import time
import threading
import json

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from std_msgs.msg import String, Float32

from sailbot import constants as c
from sailbot.utils.boatMath import distance_between
from sailbot.utils.eventUtils import Event, EventFinished
from sailbot.utils.utils import Waypoint, has_reached_waypoint

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if DOCKER == "True" else False
folder = "sailbot.peripherals." if not DOCKER else "sailbot.virtualPeripherals."

class DummyEvent(Event):
    """
    Attributes:
        - target (Waypoint): the center of the search bounds
    """

    required_args = []

    def __init__(self, event_info):
        """
        Args:
            - _event_info (list): list containing target
                - expects [Waypoint(center_lat, center_long)]
        """
        super().__init__(event_info)

        self.sail_pub = self.create_publisher(Float32, "/boat/cmd_sail", 10)
        self.rudder_pub = self.create_publisher(Float32, "/boat/cmd_rudder", 10)

    def next_gps(self):
        self.sail_pub.publish(Float32(data=0.0))
        self.rudder_pub.publish(Float32(data=100.0))
        return Waypoint(None, None)

def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/main"
    rclpy.init(args=args)

    event = DummyEvent({})

    try:
        rclpy.spin(event)

    except KeyboardInterrupt:
        print("Exiting gracefully.")

    rclpy.shutdown()