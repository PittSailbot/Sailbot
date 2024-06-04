"""
Template code for creating new events. Currently, code sails through a list of defined waypoints.

The process for creating new events:
    - Add parameters in config/params_eventDefaults.yaml
    - Initialize parameters in __init__
    - Subscribe to necessary sensors in __init__
        - Set up callback functions to update local variables
    - Add event to setup.py:entry_points
    - Add event logic to event_loop
"""
import os
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
    def __init__(self):
        super().__init__()

        # Initialize parameters from params_eventDefaults.yaml
        self.configureParameters(expected={"Waypoints": [[]]})

        self.waypoint_queue = []
        wps = list(self.get_parameter('Waypoints').value)
        for wp in wps:
            self.waypoint_queue.append(Waypoint(wp[0], wp[1]))

        # Initialize ROS subscribers & publishers (sensors, messages, etc.)
        self.next_gps_pub = self.create_subscription(String, "/boat/next_gps", 10)
        self.target = Waypoint(self.waypoint_queue.pop())

        self.gps_sub = self.create_subscription(String, "GPS", self.gps_callback, 10)
        self.position = Waypoint(0, 0)

        self.waypoint_queue_pub = self.create_publisher(String, "queued_waypoints", 10)

        # Create timer for ROS to execute 'event_loop' every 5s
        self.timer = self.create_timer(5, self.event_loop)

    def gps_callback(self, msg):
        self.position = Waypoint.from_msg(msg)

    def event_loop(self):
        """Event logic. Executed continuously by ROS's timer."""
        try:
            if distance_between(self.position, self.target) < float(c.config["CONSTANTS"]["reached_waypoint_distance"]):
                self.logging.info("Reached Waypoint")
                self.target = self.waypoint_queue.pop()

                if self.target is None:
                    raise EventFinished("Reached all waypoints")

            # Publish next gps to navigate towards
            msg = String()
            msg.data = self.target.to_msg()
            self.next_gps_pub.publish(msg)

            # Publish waypoints for sailbot website map
            msg = String()
            msg.data = json.dumps({'Waypoints': [self.target.toJson()] + [wp.toJson() for wp in self.waypoint_queue]})
            self.waypoint_queue_pub.publish(msg)
            self.logging.debug(F"{self.__class__.__name__} publishing next_GPS: {self.target}")

        except EventFinished as e:
            self.logging.info(f"{self.__class__.__name__} finished")
            pass

        except Exception as e:
            self.logging.error(F"{self.__class__.__name__} failed to get next GPS with error: {e}")
            target = String()
            target.data = ""

    def configureParameters(self, expected):
        descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            description='Latitude and longitude for the target waypoint'
        )
        default_value = None
        self.declare_parameter('Waypoints', default_value, descriptor)

        # Verify the parameters were set when the Node was created
        for param in list(expected.keys()):
            paramValue = self.get_parameter(param).value

            if paramValue is None:
                self.logging.error(F"{self.__class__.__name__} parameters not set for '{param}'!")
                raise Exception(F"{self.__class__.__name__} parameters not set for '{param}'!")


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/main"
    rclpy.init(args=args)

    event = DummyEvent()

    try:
        rclpy.spin(event)

    except KeyboardInterrupt:
        print("Exiting gracefully.")

    rclpy.shutdown()
