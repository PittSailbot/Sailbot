import importlib
import math
import os
import time
import threading
import json

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from std_msgs.msg import String, Float32, Int32

from sailbot import constants as c
from sailbot.utils.boatMath import distance_between
from sailbot.utils.eventUtils import Event, EventFinished
from sailbot.utils.utils import Waypoint, has_reached_waypoint, ControlState

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if DOCKER == "True" else False
folder = "sailbot.peripherals." if not DOCKER else "sailbot.virtualPeripherals."


class HeaveTo(Event):
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

        self.control_state_sub = self.create_subscription(String, "/boat/control_state", self.control_state_callback, 2)
        self.control_state = None

        self.start_time = time.time()

    def control_state_timer_callback(self):
        self.event_control_state.publish(Int32(data=ControlState.EXTERNAL_CONTROL))

    def control_state_callback(self, msg):
        self.control_state = ControlState.fromRosMessage(msg)

        if not self.control_state.full_auto:
            self.start_time = time.time()

    def next_gps(self):
        if self.control_state and self.control_state.full_auto:
            if time.time() > (self.start_time + 290):
                self.logging.info("Leaving")
                self.sail_pub.publish(Float32(data=0.0))
                self.rudder_pub.publish(Float32(data=0.0))
            else:
                self.logging.info(f"Heaving for {time.time() - self.start_time}s. {(self.start_time + 290) - time.time()}s left.")
                self.sail_pub.publish(Float32(data=0.0))
                self.rudder_pub.publish(Float32(data=c.config['RUDDER']['max_angle']))

        return Waypoint(None, None)


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/main"
    rclpy.init(args=args)

    event = HeaveTo({})

    try:
        rclpy.spin(event)

    except KeyboardInterrupt:
        print("Exiting gracefully.")

    rclpy.shutdown()
