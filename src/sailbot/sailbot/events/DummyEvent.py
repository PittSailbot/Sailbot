import importlib
import math
import os
import time
import threading
import json

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from std_msgs.msg import String

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

        # Declare parameters and their default values
        self.configureParameters()

        # Set data based on parameters
        target = self.get_parameter('Waypoint1').value
        self.target = Waypoint(target[0], target[1])

        self.queuedWaypoints = []
        for param in ['Waypoint2', 'Waypoint3', 'Waypoint4']:
            target = self.get_parameter(param).value
            self.queuedWaypoints.append(Waypoint(target[0], target[1]))

        self.pub_queuedWaypoints = self.create_publisher(String, "queued_waypoints", 10)
        timer_period = 10.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_queued_waypoints)

        self.set_event_subscription = self.create_subscription(String, "set_event_target", self.ROS_setEventCallback, 10)

        self.gps_sub = self.create_subscription(String, "/boat/GPS", self.gps_callback, 2)

        self.position = None

    def gps_callback(self, msg):
        self.position = Waypoint.from_msg(msg)

    def configureParameters(self):
        descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY, description='Latitude and longitude for the target waypoint')
        default_value = None
        self.declare_parameter('Waypoint1', default_value, descriptor)
        self.declare_parameter('Waypoint2', default_value, descriptor)
        self.declare_parameter('Waypoint3', default_value, descriptor)
        self.declare_parameter('Waypoint4', default_value, descriptor)

        # Verify the parameters were set when the Node was created
        paramList = ['Waypoint1', 'Waypoint2', 'Waypoint3', 'Waypoint4']
        for param in paramList:
            paramValue = self.get_parameter(param).value

            if paramValue == None:
                self.logging.error(F"{self.__class__.__name__} parameters not set!")
                raise Exception(F"{self.__class__.__name__} parameters not set!")

            if len(paramValue) != 2:
                self.logging.error(F"{self.__class__.__name__} parameters invalid!")
                raise Exception(F"{self.__class__.__name__} parameters invalid!")

    def next_gps(self):
        if self.position and distance_between(self.position, self.target) < 3:
            self.target = self.queuedWaypoints.pop(0)
        return self.target

    def publish_queued_waypoints(self):
        msg = String()
        msg.data = json.dumps({'Waypoints': [self.target.toJson()] + [wp.toJson() for wp in self.queuedWaypoints]})
        self.pub_queuedWaypoints.publish(msg)

    def ROS_setEventCallback(self, msg):
        self.target = Waypoint.from_msg(msg)


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/main"
    rclpy.init(args=args)

    event = DummyEvent({})

    try:
        rclpy.spin(event)

    except KeyboardInterrupt:
        print("Exiting gracefully.")

    rclpy.shutdown()
