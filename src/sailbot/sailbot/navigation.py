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
from sailbot.utils.utils import Waypoint, ControlState, ImuData

TACK_TO_WIND_STARBOARD = -1
TACK_TO_WIND_PORT = 1

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

    def __init__(self):
        super().__init__("navigation")
        self.logging = self.get_logger()

        self.position = Waypoint(0, 0)
        self.compass_angle = 0
        self.wind_angle = 0
        self.imuData = None
        self.boat_speed = float('inf') # change this to boat speed once available
        self.aborted_tacks = 0

        self.next_gps_sub = self.create_subscription(String, "/boat/next_gps", self.next_gps_callback, 2)
        self.gps_sub = self.create_subscription(String, "/boat/GPS", self.gps_callback, 2)
        self.imu_sub = self.create_subscription(String, "/boat/imu", self.imu_callback, 2)
        self.windvane_sub = self.create_subscription(String, "/boat/wind_angle", self.windvane_callback, 2)
        self.control_state_sub = self.create_subscription(String, "/boat/control_state", self.control_state_callback, 2)

        self.sail_pub = self.create_publisher(Float32, "/boat/cmd_sail", 10)
        self.rudder_pub = self.create_publisher(Float32, "/boat/cmd_rudder", 10)

        self.go_to_gps_timer = self.create_timer(0.2, self.go_to_gps)
        self.sail_adjust_timer = self.create_timer(0.5, self.auto_adjust_sail)

        self.latest_waypoint = None
        self.control_state = None
        self.allow_tacking = True
        self.tack = None

    def next_gps_callback(self, msg):
        next_gps = Waypoint.from_msg(msg)
        if next_gps != self.latest_waypoint:
            self.logging.info(f"Navigating to {next_gps}")
            self.latest_waypoint = next_gps

    def control_state_callback(self, msg):
        self.control_state = ControlState.fromRosMessage(msg)

    def windvane_callback(self, msg):
        angle = float(msg.data)
        self.wind_angle = angle % 360

    def imu_callback(self, msg):
        self.imuData = ImuData.fromRosMessage(msg)
        self.compass_angle = (self.imuData.yaw) % 360

    def gps_callback(self, msg):
        self.position = Waypoint.from_msg(msg)

    def go_to_gps(self):
        """
        Moves the boat to the target GPS
        Args:
            target (Waypoint): the GPS point to go to
        """
        target = self.latest_waypoint

        if self.control_state == None or self.control_state.rudder != ControlState.AUTO or target is None:
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

        # Boat can't sail straight upwind; snap angle to the closest allowed no_go_zone bound if target angle is in irons
        # TODO take into account angular velocity
        no_go_zone_left_bound, no_go_zone_right_bound = boatMath.get_no_go_zone_bounds(self.wind_angle, self.compass_angle)
        if self.tack == None and boatMath.is_within_angle(self.compass_angle, no_go_zone_left_bound, no_go_zone_right_bound):
            self.logging.warning("Unexpectedly in irons, turning to nearest edge")
            
            if boatMath.degrees_between(target_angle, no_go_zone_left_bound) < boatMath.degrees_between(target_angle, no_go_zone_right_bound):
                self.tack = TACK_TO_WIND_STARBOARD
            else:
                self.tack = TACK_TO_WIND_PORT

        elif self.tack == None and boatMath.is_within_angle(target_angle, no_go_zone_left_bound, no_go_zone_right_bound):
            # Need to go upwind
            # Stick to the closest angle until we can sail directly to target
            
            if boatMath.degrees_between(self.compass_angle, no_go_zone_left_bound) < boatMath.degrees_between(self.compass_angle, no_go_zone_right_bound):
                target_angle = no_go_zone_left_bound
            else:
                target_angle = no_go_zone_right_bound

            self.logging.info(F"Need to go upwind, setting target angle to {target_angle}")
        
        
        if self.tack != None:
            # turn as fast as possible to complete tack
            self.complete_tack()
        else:
            self.turn_to_angle(target_angle)

    def turn_to_angle(self, target_angle):
        """
        Sets the rudder once to turn the board towards the specified compass angle
            - Does NOT recenter the rudder once it faces the specified angle; only checked when function is called
        Args:
            target_angle (float): the angle to turn to
            allow_tacking (bool): whether the boat is allowed to turn through the 'no go zone'
        """
        if boatMath.is_within_angle(target_angle, (self.compass_angle - self.ACCEPTABLE_ERROR) % 360, (self.compass_angle + self.ACCEPTABLE_ERROR) % 360):
            self.logging.debug(f"Holding boat at {target_angle} degrees")
            msg = Float32()
            msg.data = 0.0
            self.rudder_pub.publish(msg)
            return

        no_go_zone_center = self.wind_angle
        turning_right = (self.compass_angle - target_angle) % 360 < 180
        is_no_go_zone_right = ((self.wind_angle + self.compass_angle) - no_go_zone_center) % 360 < 180
        no_go_distance = boatMath.degrees_between(self.compass_angle, no_go_zone_center)
        distance_to_target = boatMath.degrees_between(self.compass_angle, target_angle)

        need_to_tack = turning_right and is_no_go_zone_right and distance_to_target > no_go_distance
        need_to_tack = need_to_tack or (not turning_right and not is_no_go_zone_right and distance_to_target > no_go_distance)

        if (need_to_tack):
            # Shortest path to target is across the no-go-zone
            if self.allow_tacking:
                # TODO: Check speed before committing to a tack
                self.tack = TACK_TO_WIND_STARBOARD if turning_right else TACK_TO_WIND_PORT
                self.logging.info(F"Starting tack: {self.tack}")
                return self.complete_tack()
            else:
                # Jibe to target
                if ((self.compass_angle < target_angle) and (self.compass_angle <= self.wind_angle <= target_angle)) or \
                    ((target_angle < self.compass_angle) and (target_angle >= self.wind_angle and self.wind_angle <= self.compass_angle)):
                    self.logging.debug(f"Jibing left from {self.compass_angle} degrees to {target_angle} degrees")
                    rudder_angle = -self.SMOOTHING_CONSTANT * abs(self.compass_angle - target_angle)
                else:
                    self.logging.debug(f"Jibing right from {self.compass_angle} degrees to {target_angle} degrees")
                    rudder_angle = self.SMOOTHING_CONSTANT * abs(self.compass_angle - target_angle)

        elif turning_right:
            self.logging.debug(f"Turning right from {self.compass_angle} degrees to {target_angle} degrees")
            rudder_angle = self.SMOOTHING_CONSTANT * abs(self.compass_angle - target_angle)
        else:
            self.logging.debug(f"Turning left from {self.compass_angle} degrees to {target_angle} degrees")
            rudder_angle = -self.SMOOTHING_CONSTANT * abs(self.compass_angle - target_angle)

        if abs(rudder_angle) > 90:
            self.logging.warning(F"Navigation suggested rudder angle is very large ({rudder_angle})")
        
        rudder_angle = min(rudder_angle, float(c.config['RUDDER']['max_angle']))
        rudder_angle = max(rudder_angle, float(c.config['RUDDER']['min_angle']))

        msg = Float32()
        msg.data = rudder_angle
        self.rudder_pub.publish(msg)

    def complete_tack(self):
        no_go_zone_left_bound, no_go_zone_right_bound = boatMath.get_no_go_zone_bounds(self.wind_angle, self.compass_angle)

        if boatMath.degrees_between(self.compass_angle, no_go_zone_left_bound) < boatMath.degrees_between(self.compass_angle, no_go_zone_right_bound):
            closest = TACK_TO_WIND_STARBOARD
        else:
            closest = TACK_TO_WIND_PORT
            
        if self.boat_speed <= float(c.config['NAVIGATION']['tack_min_continue_speed']) and self.tack != closest:
            # abort tack
            self.logging.warning("Speed dropped too rapidly, aborting tack")
            self.aborted_tacks += 1
            self.tack = closest

        if (not boatMath.is_within_angle(self.compass_angle, no_go_zone_left_bound, no_go_zone_right_bound) and closest == self.tack):
            # tack complete
            self.logging.info("Tack Complete")
            self.tack = None
            return

        else:
            if self.tack == TACK_TO_WIND_STARBOARD:
                rudder_angle = float(c.config['RUDDER']['max_angle'])
            else:
                rudder_angle = float(c.config['RUDDER']['min_angle'])

        msg = Float32()
        msg.data = rudder_angle
        self.rudder_pub.publish(msg)
        self.logging.debug("continuing tack")

    def auto_adjust_sail(self):
        """Adjusts the sail to the optimal angle for speed"""
        if self.control_state == None or self.control_state.sail != ControlState.AUTO:
            return 
        
        if self.wind_angle > 180:
            wind_angle = 180 - (self.wind_angle - 180)
        else:
            wind_angle = self.wind_angle

        sail_angle = max(min(wind_angle / 2, 90), 3)

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
