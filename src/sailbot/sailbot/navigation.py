"""
Handles autonomous navigation to a desired GPS point
"""
import logging
import os
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

from sailbot import constants as c
from sailbot.utils import boatMath, utils
from sailbot.utils.utils import Waypoint, ControlState


# TODO: Implement Waypoint from serialized ROS message
class Navigation(Node):
    """Autonomously navigates the boat to a desired GPS waypoint
        Listens to:
            - /next_gps (Waypoint): The next GPS point to navigate to
            - /autonomy_enabled (bool??):
            - /gps (String):
            - /compass (String):
            - /windvane (String):
            - /control_state (Bool):
        Publishes to:
            - /cmd_rudder (String)
            - /cmd_sail (String)
    """
    ACCEPTABLE_ERROR = float(c.config["RUDDER"]["acceptable_error"])
    SMOOTHING_CONSTANT = float(c.config["RUDDER"]["smooth_const"])
    NO_GO_ANGLE = float(c.config["NAVIGATION"]["no_go_angle"])

    def __init__(self):
        super().__init__("navigation")
        self.logging = self.get_logger()

        self.position = Waypoint(0, 0)
        self.compass_angle = 0
        self.wind_angle = 0

        self.next_gps_sub = self.create_subscription(String, "/boat/next_gps", self.next_gps_callback, 2)
        self.gps_sub = self.create_subscription(String, "/boat/GPS", self.gps_callback, 2)
        self.compass_sub = self.create_subscription(String, "/boat/compass", self.compass_callback, 2)
        self.windvane_sub = self.create_subscription(String, "/boat/windvane", self.windvane_callback, 2)
        self.control_state_pub = self.create_subscription(String, "/boat/control_state", self.control_state_callback, 2)

        self.sail_pub = self.create_publisher(Float32, "/boat/cmd_sail", 10)
        self.rudder_pub = self.create_publisher(Float32, "/boat/cmd_rudder", 10)

        self.go_to_gps_timer = self.create_timer(0.2, self.go_to_gps)
        self.sail_adjust_timer = self.create_timer(0.5, self.auto_adjust_sail)

        self.latest_waypoint = None
        self.control_state = None
        self.allow_tacking = True

    def next_gps_callback(self, msg):
        next_gps = Waypoint.from_string(msg)
        if next_gps != self.latest_waypoint:
            self.logging.info(f"Navigating to {next_gps}")
            self.latest_waypoint = next_gps

    def control_state_callback(self, msg):
        self.control_state = ControlState.fromRosMessage(msg)

    def windvane_callback(self, msg):
        data_dict = json.loads(msg.data)
        self.wind_angle = float(data_dict['angle'])

    def compass_callback(self, msg):
        self.compass_angle = float(msg.data)

    def gps_callback(self, msg):
        self.position = Waypoint.from_gps_msg(msg)

    def go_to_gps(self):
        """
        Moves the boat to the target GPS
        Args:
            target (Waypoint): the GPS point to go to
        """
        target = self.latest_waypoint

        if self.control_state == None or self.control_state.rudder_manual or target is None:
            return

        if utils.has_reached_waypoint(target):
            self.logging.info(f"Reached {target}")
            self.latest_waypoint = None
            self.sail_pub.publish(0)
            self.rudder_pub.publish(0)
            return

        target_angle = boatMath.angleToPoint(self.position.lat, self.position.lon, target.lat, target.lon)
        delta_angle = (target_angle - self.compass_angle) % 360
        if delta_angle > 180:
            delta_angle -= 360

        self.logging.info(F"Nav- target_angle: {target_angle}, delta_angle: {delta_angle}")

        # TODO: Fix this its not working
        # Boat can't sail straight upwind; snap angle to the closest allowed no_go_zone bound if target angle is in irons
        # no_go_zone_left_bound = (self.wind_angle - self.NO_GO_ANGLE / 2) % 360
        # no_go_zone_right_bound = (self.wind_angle + self.NO_GO_ANGLE / 2) % 360
        # if boatMath.is_within_angle(target_angle, no_go_zone_left_bound, no_go_zone_right_bound):
        #     self.logging.info("Cannot sail directly to point")
        #     if (delta_angle + self.wind_angle) % 360 < no_go_zone_left_bound:
        #         target_angle = no_go_zone_left_bound
        #     elif (delta_angle + self.wind_angle) % 360 < no_go_zone_right_bound:
        #         target_angle = no_go_zone_right_bound

        # TODO: Check speed before commiting to a tack
        self.turn_to_angle(target_angle, self.allow_tacking)

    def turn_to_angle(self, target_angle, allow_tacking=True):
        """
        Sets the rudder once to turn the board towards the specified compass angle
            - Does NOT recenter the rudder once it faces the specified angle; only checked when function is called
        Args:
            target_angle (float): the angle to turn to
            allow_tacking (bool): whether the boat is allowed to turn through the 'no go zone'
        """
        if boatMath.is_within_angle(target_angle, (self.compass_angle - self.ACCEPTABLE_ERROR) % 360, (self.compass_angle + self.ACCEPTABLE_ERROR) % 360):
            self.logging.info(f"Holding boat at {target_angle} degrees")
            msg = Float32()
            msg.data = 0.0
            self.rudder_pub.publish(msg)
            return

        if allow_tacking:
            if (self.compass_angle - target_angle) % 360 < 180:
                self.logging.info(f"Tacking right from {self.compass_angle} degrees to {target_angle} degrees")
                rudder_angle = self.SMOOTHING_CONSTANT * abs(self.compass_angle - target_angle)
            else:
                self.logging.info(f"Tacking left from {self.compass_angle} degrees to {target_angle} degrees")
                rudder_angle = -self.SMOOTHING_CONSTANT * abs(self.compass_angle - target_angle)
        else:
            if ((self.compass_angle < target_angle) and (self.compass_angle <= self.wind_angle <= target_angle)) or \
                    ((target_angle < self.compass_angle) and (target_angle >= self.wind_angle and self.wind_angle <= self.compass_angle)):
                self.logging.info(f"Jibing left from {self.compass_angle} degrees to {target_angle} degrees")
                rudder_angle = -self.SMOOTHING_CONSTANT * abs(self.compass_angle - target_angle)
            else:
                self.logging.info(f"Jibing right from {self.compass_angle} degrees to {target_angle} degrees")
                rudder_angle = self.SMOOTHING_CONSTANT * abs(self.compass_angle - target_angle)

        if abs(rudder_angle) > 90:
            self.logging.warning(F"Navigation suggested rudder angle is very large ({rudder_angle})")
        
        rudder_angle = min(rudder_angle, float(c.config['RUDDER']['max_angle']))
        rudder_angle = max(rudder_angle, float(c.config['RUDDER']['min_angle']))

        msg = Float32()
        msg.data = rudder_angle
        self.rudder_pub.publish(msg)

    def auto_adjust_sail(self):
        """Adjusts the sail to the optimal angle for speed"""
        if self.control_state == None or self.control_state.sail_manual:
            return 
        
        if self.wind_angle > 180:
            self.wind_angle = 180 - (self.wind_angle - 180)

        sail_angle = max(min(self.wind_angle / 2, 90), 3)

        msg = Float32()
        msg.data = float(sail_angle)
        self.sail_pub.publish(msg)


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/navigation"
    rclpy.init(args=args)

    navigation = Navigation()
    rclpy.spin(navigation)


if __name__ == "__main__":
    main()
