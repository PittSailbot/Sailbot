"""
Handles autonomous navigation to a desired GPS point
"""
import logging
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sailbot import constants as c
from sailbot.utils import boatMath, utils
from sailbot.utils.eventUtils import Waypoint


# TODO: Interrupt on new data and RC enabled. Use service or separate callback?
# TODO: Implement Waypoint from serialized ROS message
# TODO: Fix or remove windvane.no_go_zone
class Navigation(Node):
    """Autonomously navigates the boat to a desired GPS waypoint
        Listens to:
            - /next_gps (Waypoint???): The next GPS point to navigate to
            - /autonomy_enabled (bool??):
            - /gps (String):
            - /compass (String):
            - /windvane (String):
        Publishes to:
            - /cmd_rudder (String)
            - /cmd_sail (String)
    """
    ACCEPTABLE_ERROR = float(c.config["RUDDER"]["acceptable_error"])
    SMOOTHING_CONSTANT = float(c.config["RUDDER"]["smooth_const"])

    def __init__(self):
        super().__init__("navigation")
        self.logging = self.get_logger()

        self.position = 0
        self.compass_angle = 0
        self.wind_angle = 0

        self.next_gps_callback = self.create_subscription(String, "next_gps", self.next_gps_callback, 10)
        self.interrupt_callback = self.create_subscription(Bool, "autonomy_enabled", self.interrupt_callback, 10)
        self.gps_callback = self.create_subscription(String, "gps", self.gps_callback, 10)
        self.compass_callback = self.create_subscription(String, "compass", self.compass_callback, 10)
        self.windvane_callback = self.create_subscription(String, "windvane", self.windvane_callback, 10)

        self.sail_pub = self.create_publisher(String, "cmd_sail", 10)
        self.rudder_pub = self.create_publisher(String, "cmd_rudder", 10)

    def next_gps_callback(self, msg):  # TODO: needs to be changed from callback to action server https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html
        next_gps = Waypoint.from_bytes(msg)
        self.logging.info(f"Navigating to {next_gps}")
        self.go_to_gps(next_gps)
        # TODO: Interrupt
        pass

    def windvane_callback(self, msg):
        self.wind_angle = int(msg)

    def compass_callback(self, msg):
        self.compass_angle = int(msg)

    def gps_callback(self, msg):
        self.position = Waypoint.from_bytes(msg)

    def go_to_gps(self, waypoint, wait_until_finished=False):
        """
        Moves the boat to the target GPS
        Args:
            waypoint (Waypoint): the GPS point to go to
            wait_until_finished (bool): whether to wait until the boat has reached the GPS point
        Raises:

        """
        allow_tacking = True
        # TODO: interrupt logic
        while not utils.has_reached_waypoint(waypoint) and not self.interrupt and wait_until_finished:
            # determine angle we need to turn
            delta_angle = boatMath.angleToPoint(self.compass_angle, self.position.lat, self.position.lon, waypoint.lat, waypoint.lon)
            target_angle = (self.compass_angle + delta_angle) % 360

            if (delta_angle + self.wind_angle) % 360 < windvane.no_go_zone.left_bound:
                target_angle = windvane.no_go_zone.left_bound
            elif (delta_angle + self.wind_angle) % 360 < windvane.no_go_zone.right_bound:
                target_angle = windvane.no_go_zone.right_bound

            try:
                self.turn_to_angle(target_angle, allow_tacking)
            except FailedTurn:
                self.logging.warning("Failed to tack through the wind! Falling back to jibing only for this GPS.")
                allow_tacking = False

    def turn_to_angle(self, target_angle, allow_tacking=True, wait_until_finished=False):
        """
        Turns the boat using the rudder until it is facing the specified compass angle

        Args:
            target_angle (float): the angle to turn to
            allow_tacking (bool): whether the boat is allowed to turn through the 'no go zone'
            wait_until_finished (bool): whether to wait until the boat has reached the angle
                # TODO: when false, the rudder won't be reset after the angle is reached, NEEDS TO SPAWN A THREAD
                    that thread must also be deleted if another turn_to_angle is called before angle is reached
        Raises:
            - FailedTurn (Exception): the boat failed to turn to the specified angle for whatever reason (probably speed)
        """
        logging.info(f"Turning boat from {self.compass_angle} degrees to {target_angle} degrees")
        while True:
            self.auto_adjust_sail()

            if abs(self.compass_angle - target_angle) > self.ACCEPTABLE_ERROR:
                logging.info(f"Finished turning boat to {target_angle} degrees")
                break

            if allow_tacking:
                if (self.compass_angle - target_angle) % 360 < 180:
                    rudder_angle = self.SMOOTHING_CONSTANT * abs(self.compass_angle - target_angle)  # Turn right
                else:
                    rudder_angle = -self.SMOOTHING_CONSTANT * abs(self.compass_angle - target_angle)  # Turn left
            else:
                if ((self.compass_angle < target_angle) and self.compass_angle <= self.wind_angle <= target_angle) or (
                        target_angle < self.compass_angle and (target_angle >= self.wind_angle and self.wind_angle <= self.compass_angle)):
                    rudder_angle = -self.SMOOTHING_CONSTANT * abs(self.compass_angle - target_angle)  # Turn left
                else:
                    rudder_angle = self.SMOOTHING_CONSTANT * abs(self.compass_angle - target_angle)  # Turn right

            self.rudder_pub().publish(rudder_angle)

            if not wait_until_finished:
                break

    def auto_adjust_sail(self):
        """Adjusts the sail to the optimal angle for speed"""
        if self.wind_angle > 180:
            self.wind_angle = 180 - (self.wind_angle - 180)

        sail_angle = max(min(self.wind_angle / 2, 90), 3)

        self.sail_pub().publish(sail_angle)


class FailedTurn(RuntimeError):
    """Signals that the boat failed to turn to the proper angle, usually indicates a failed tack"""
    pass


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/navigation"
    rclpy.init(args=args)

    navigation = Navigation()
    rclpy.spin(navigation)


if __name__ == "__main__":
    main()
