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

        self.auto_sail_pub = self.create_publisher(Float32, "/boat/cmd_auto_sail", 10)
        self.sail_pub = self.create_publisher(Float32, "/boat/cmd_sail", 10)
        self.auto_rudder_pub = self.create_publisher(Float32, "/boat/cmd_auto_rudder", 10)
        self.rudder_pub = self.create_publisher(Float32, "/boat/cmd_rudder", 10)

        self.go_to_gps_timer = self.create_timer(0.2, self.go_to_gps)
        self.sail_adjust_timer = self.create_timer(0.5, self.auto_adjust_sail)

        self.latest_waypoint = None
        self.control_state = None
        self.allow_tacking = False
        self.tack = None
        self.calculate_autonomy_always = True
        self.emergency_tack = False

        self.rudderMult = 0.33

        self.jibe_angle_increase = 75 # (degrees)

    @property
    def manualSails(self):
        return self.control_state == None or self.control_state.sail != ControlState.AUTO

    @property
    def manualRudder(self):
        return self.control_state == None or self.control_state.rudder != ControlState.AUTO or self.latest_waypoint is None


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
        if target == None or (self.manualRudder and not self.calculate_autonomy_always):
            return
        
        if boatMath.distance_between(self.position, target) < float(c.config["CONSTANTS"]["reached_waypoint_distance"]):
            self.logging.info(f"Reached {target}. Heaving.")
            self.latest_waypoint = None
            self.auto_sail_pub.publish(Float32(data=0.0))
            self.auto_rudder_pub.publish(Float32(data=0.0))
            if not self.manualRudder:
                self.sail_pub.publish(Float32(data=0.0))
                self.rudder_pub.publish(Float32(data=0.0))
            return

        target_angle = boatMath.angleToPoint(self.position.lat, self.position.lon, target.lat, target.lon)
        delta_angle = (target_angle - self.compass_angle) % 360
        if delta_angle > 180:
            delta_angle -= 360

        # Boat can't sail straight upwind; snap angle to the closest allowed no_go_zone bound if target angle is in irons
        # # TODO take into account angular velocity
        no_go_zone_left_bound, no_go_zone_right_bound = boatMath.get_no_go_zone_bounds(self.wind_angle, self.compass_angle)

        if self.tack is None and boatMath.is_within_angle(self.compass_angle, no_go_zone_left_bound, no_go_zone_right_bound):
            self.logging.warning("Unexpectedly in irons, turning to nearest edge")
            self.emergency_tack = True
            if boatMath.degrees_between(target_angle, no_go_zone_left_bound) < boatMath.degrees_between(target_angle, no_go_zone_right_bound):
                self.tack = TACK_TO_WIND_PORT
            else:
                self.tack = TACK_TO_WIND_STARBOARD

        elif self.tack is None and boatMath.is_within_angle(target_angle, no_go_zone_left_bound, no_go_zone_right_bound):
            # Need to go upwind
            # Stick to the closest angle until we can sail directly to target
            
            if boatMath.degrees_between(self.compass_angle, no_go_zone_left_bound) < boatMath.degrees_between(self.compass_angle, no_go_zone_right_bound):
                target_angle = no_go_zone_left_bound
            else:
                target_angle = no_go_zone_right_bound

            self.logging.info(F"Need to go upwind, setting target angle to {target_angle}")

        if self.tack is not None:
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
            self.logging.info(f"Holding boat at {target_angle} degrees")
            msg = Float32()
            msg.data = 0.0 * self.rudderMult
            self.auto_rudder_pub.publish(msg)
            if not self.manualRudder:
                self.rudder_pub.publish(msg)
            return

        no_go_zone_center = self.wind_angle + self.compass_angle
        turning_left = (self.compass_angle - target_angle) % 360 < 180
        is_no_go_zone_left = (self.wind_angle - 180) > 0
        no_go_distance = boatMath.degrees_between(self.compass_angle, no_go_zone_center)
        if not self.allow_tacking:
            no_go_distance += self.jibe_angle_increase
        distance_to_target = boatMath.degrees_between(self.compass_angle, target_angle)

        self.logging.info(str((turning_left, is_no_go_zone_left, distance_to_target, no_go_distance)))
        need_to_tack = not turning_left and not is_no_go_zone_left and distance_to_target > no_go_distance
        need_to_tack = need_to_tack or (turning_left and is_no_go_zone_left and distance_to_target > no_go_distance)

        self.logging.info(str((turning_left, is_no_go_zone_left, distance_to_target, no_go_distance)))
        self.logging.info(str(need_to_tack))

        if (need_to_tack):
            # Shortest path to target is across the no-go-zone
            self.tack = TACK_TO_WIND_STARBOARD if turning_left else TACK_TO_WIND_PORT
            tack_or_jibe = "Tack" if self.allow_tacking else "Jibe"
            self.logging.info(F"Starting {tack_or_jibe}: {self.tack}")
            return self.complete_tack()
        
        no_go_zone_left_bound, no_go_zone_right_bound = boatMath.get_no_go_zone_bounds(self.wind_angle, self.compass_angle)
        if boatMath.degrees_between(self.compass_angle, no_go_zone_left_bound) < boatMath.degrees_between(self.compass_angle, no_go_zone_right_bound):
            target_angle = no_go_zone_left_bound
        else:
            target_angle = no_go_zone_right_bound

        if turning_left:
            self.logging.info(f"Turning left from {self.compass_angle} degrees to {target_angle} degrees")
            rudder_angle = self.SMOOTHING_CONSTANT * abs(self.compass_angle - target_angle)
        else:
            self.logging.info(f"Turning right from {self.compass_angle} degrees to {target_angle} degrees")
            rudder_angle = -self.SMOOTHING_CONSTANT * abs(self.compass_angle - target_angle)

        rudder_angle = min(rudder_angle, float(c.config['RUDDER']['max_angle']))
        rudder_angle = max(rudder_angle, float(c.config['RUDDER']['min_angle']))
        
        msg = Float32()
        msg.data = rudder_angle * self.rudderMult
        self.auto_rudder_pub.publish(msg)
        if not self.manualRudder:
            self.rudder_pub.publish(msg)

    def complete_tack(self):
        no_go_zone_left_bound = (self.wind_angle - float(c.config["WINDVANE"]["no_go_range"])) % 360
        no_go_zone_right_bound = (self.wind_angle + float(c.config["WINDVANE"]["no_go_range"])) % 360

        tack_or_jibe = "Tack" if self.allow_tacking else "Jibe"

        if boatMath.degrees_between(self.compass_angle, no_go_zone_left_bound) < boatMath.degrees_between(self.compass_angle, no_go_zone_right_bound):
            closest = TACK_TO_WIND_STARBOARD
        else:
            closest = TACK_TO_WIND_PORT

        if self.allow_tacking or self.emergency_tack:   
                 
            if self.boat_speed <= float(c.config['NAVIGATION']['tack_min_continue_speed']) and self.tack != closest:
                # abort tack
                self.logging.warning("Speed dropped too rapidly, aborting tack")
                self.aborted_tacks += 1
                self.tack = closest

            else:
                if self.tack == TACK_TO_WIND_STARBOARD:
                    rudder_angle = float(c.config['RUDDER']['max_angle'])
                else:
                    rudder_angle = float(c.config['RUDDER']['min_angle'])

            if not boatMath.is_within_angle(0, no_go_zone_left_bound, no_go_zone_right_bound) and closest == self.tack:
                # tack complete
                self.logging.info(F"{tack_or_jibe} Complete")
                self.emergency_tack = False
                self.tack = None

        else:
            if self.tack == TACK_TO_WIND_STARBOARD:
                rudder_angle = float(c.config['RUDDER']['min_angle'])
            else:
                rudder_angle = float(c.config['RUDDER']['max_angle'])

            if (self.tack == TACK_TO_WIND_STARBOARD and (self.wind_angle) < 180) or self.tack == TACK_TO_WIND_PORT and (self.wind_angle > 180):
                # tack complete
                self.logging.info(F"{tack_or_jibe} Complete")
                self.emergency_tack = False
                self.tack = None

        msg = Float32()
        msg.data = rudder_angle * self.rudderMult
        self.auto_rudder_pub.publish(msg)
        if not self.manualRudder:
            self.rudder_pub.publish(msg)

        self.logging.info(F"continuing {tack_or_jibe}")

    def auto_adjust_sail(self):
        """Adjusts the sail to the optimal angle for speed"""

        if self.manualSails and not self.calculate_autonomy_always:
            return

        if self.wind_angle > 180:
            wind_angle = 180 - (self.wind_angle - 180)
        else:
            wind_angle = self.wind_angle

        sail_angle = max(min(wind_angle / 2, 90), 3)

        msg = Float32()
        msg.data = float(sail_angle)
        self.auto_sail_pub.publish(msg)
        if not self.manualSails:
            self.sail_pub.publish(msg)


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/navigation"
    rclpy.init(args=args)

    navigation = Navigation()
    rclpy.spin(navigation)


if __name__ == "__main__":
    main()
