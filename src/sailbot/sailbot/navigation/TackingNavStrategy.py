"""
Handles autonomous navigation to a desired GPS point
"""

import os
from enum import Enum

import rclpy
from std_msgs.msg import Float32, String

from sailbot import constants as c
from sailbot.navigation.navStrategy import NavigationStrategy
from sailbot.utils import boatMath, utils


class Mode(Enum):
    TACKING_STARBOARD = 1
    TACKING_PORT = 2


# TODO: Implement Waypoint from serialized ROS message
class TackingNavigationStrategy(NavigationStrategy):
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

    def __init__(self):
        self.boat_speed = float("inf")  # change this to boat speed once available
        self.aborted_tacks = 0

        self.latest_waypoint = None
        self.control_state = None
        self.allow_tacking = True
        self.mode = None
        self.emergency_tack = False

        self.rudderMult = 0.33

        self.jibe_angle_increase = 75  # (degrees)

    def tick(self):
        """Update Navigation decision-making, setting sail/jib/rudder"""
        target = self.latest_waypoint

        if boatMath.distance_between(self.position, target) < float(c.config["CONSTANTS"]["reached_waypoint_distance"]):
            self.logging.info(f"Reached {target}. Heaving.")
            self.latest_waypoint = None
            self.cmd_sail_pub.publish(Float32(data=0.0))  # TODO: make heave to
            self.cmd_rudder_pub.publish(Float32(data=0.0))

        self.go_to_gps(target)

    def go_to_gps(self, target):
        """
        Moves the boat to the target GPS
        Args:
            target (Waypoint): the GPS point to go to
        """
        target_angle = boatMath.angle_to_point(self.position.lat, self.position.lon, target.lat, target.lon)
        delta_angle = (target_angle - self.compass_angle) % 360
        if delta_angle > 180:
            delta_angle -= 360

        # Boat can't sail straight upwind; snap angle to the closest allowed no_go_zone bound if target angle is in irons
        # # TODO take into account angular velocity
        if self.mode is None and boatMath.is_within_angle(self.compass_angle, self.no_go_zone_left_bound, self.no_go_zone_right_bound):
            self.logging.warning("Unexpectedly in irons, turning to nearest edge")
            self.emergency_tack = True
            if boatMath.degrees_between(target_angle, self.no_go_zone_left_bound) < boatMath.degrees_between(target_angle, self.no_go_zone_right_bound):
                self.mode = Mode.TACKING_PORT
            else:
                self.mode = Mode.TACKING_STARBOARD

        elif self.mode is None and boatMath.is_within_angle(target_angle, self.no_go_zone_left_bound, self.no_go_zone_right_bound):
            # Need to go upwind
            # Stick to the closest angle until we can sail directly to target

            if boatMath.degrees_between(self.compass_angle, self.no_go_zone_left_bound) < boatMath.degrees_between(self.compass_angle, self.no_go_zone_right_bound):
                target_angle = self.no_go_zone_left_bound
            else:
                target_angle = self.no_go_zone_right_bound

            self.logging.info(f"Need to go upwind, setting target angle to {target_angle}")

        if self.mode is not None:
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
            msg.data = self.RUDDER_CENTER
            self.cmd_rudder_pub.publish(msg)
            return

        no_go_zone_center = self.wind_angle + self.compass_angle
        turning_left = (self.compass_angle - target_angle) % 360 < 180
        is_no_go_zone_left = (self.wind_angle - 180) > 0
        no_go_distance = boatMath.degrees_between(self.compass_angle, no_go_zone_center)
        if not self.allow_tacking:
            no_go_distance += self.jibe_angle_increase
        distance_to_target = boatMath.degrees_between(self.compass_angle, target_angle)

        self.logging.info(str((turning_left, is_no_go_zone_left, distance_to_target, no_go_distance)), throttle_duration_sec=5)
        need_to_tack = not turning_left and not is_no_go_zone_left and distance_to_target > no_go_distance
        need_to_tack = need_to_tack or (turning_left and is_no_go_zone_left and distance_to_target > no_go_distance)

        self.logging.info(str((turning_left, is_no_go_zone_left, distance_to_target, no_go_distance)), throttle_duration_sec=5)
        self.logging.info(str(need_to_tack), throttle_duration_sec=5)

        if need_to_tack:
            # Shortest path to target is across the no-go-zone
            self.mode = Mode.TACKING_STARBOARD if turning_left else Mode.TACKING_PORT
            tack_or_jibe = "Tack" if self.allow_tacking else "Jibe"
            self.logging.info(f"Starting {tack_or_jibe}: {self.mode}")
            return self.complete_tack()

        no_go_zone_left_bound, self.no_go_zone_right_bound = boatMath.get_no_go_zone_bounds(self.wind_angle, self.compass_angle)
        if boatMath.degrees_between(self.compass_angle, no_go_zone_left_bound) < boatMath.degrees_between(self.compass_angle, self.no_go_zone_right_bound):
            target_angle = no_go_zone_left_bound
        else:
            target_angle = self.no_go_zone_right_bound

        if turning_left:
            self.logging.info(f"Turning left from {self.compass_angle} degrees to {target_angle} degrees", throttle_duration_sec=1)
            rudder_angle = self.SMOOTHING_CONSTANT * abs(self.compass_angle - target_angle)
        else:
            self.logging.info(f"Turning right from {self.compass_angle} degrees to {target_angle} degrees", throttle_duration_sec=1)
            rudder_angle = -self.SMOOTHING_CONSTANT * abs(self.compass_angle - target_angle)

        rudder_angle = min(rudder_angle, float(c.config["RUDDER"]["max_angle"]))
        rudder_angle = max(rudder_angle, float(c.config["RUDDER"]["min_angle"]))

        msg = Float32()
        msg.data = rudder_angle * self.rudderMult
        self.cmd_rudder_pub.publish(msg)

    def complete_tack(self):
        tack_or_jibe = "Tack" if self.allow_tacking else "Jibe"

        if boatMath.degrees_between(self.compass_angle, self.no_go_zone_left_bound) < boatMath.degrees_between(self.compass_angle, self.no_go_zone_right_bound):
            closest = Mode.TACKING_STARBOARD
        else:
            closest = Mode.TACKING_PORT

        if self.allow_tacking or self.emergency_tack:
            if self.boat_speed <= float(c.config["NAVIGATION"]["tack_min_continue_speed"]) and self.mode != closest:
                # abort tack
                self.logging.warning("Speed dropped too rapidly, aborting tack")
                self.aborted_tacks += 1
                self.mode = closest

            else:
                if self.mode == Mode.TACKING_STARBOARD:
                    rudder_angle = float(c.config["RUDDER"]["max_angle"])
                else:
                    rudder_angle = float(c.config["RUDDER"]["min_angle"])

            if not boatMath.is_within_angle(0, self.no_go_zone_left_bound, self.no_go_zone_right_bound) and closest == self.mode:
                # tack complete
                self.logging.info(f"{tack_or_jibe} Complete")
                self.emergency_tack = False
                self.mode = None

        else:
            if self.mode == Mode.TACKING_STARBOARD:
                rudder_angle = float(c.config["RUDDER"]["min_angle"])
            else:
                rudder_angle = float(c.config["RUDDER"]["max_angle"])

            if (self.mode == Mode.TACKING_STARBOARD and (self.wind_angle) < 180) or self.mode == Mode.TACKING_PORT and (self.wind_angle > 180):
                # tack complete
                self.logging.info(f"{tack_or_jibe} Complete")
                self.emergency_tack = False
                self.mode = None

        msg = Float32()
        msg.data = rudder_angle * self.rudderMult
        self.cmd_rudder_pub.publish(msg)

        self.logging.info(f"continuing {tack_or_jibe}", throttle_duration_sec=5)


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/navigation"
    rclpy.init(args=args)

    navigation = TackingNavigationStrategy()
    rclpy.spin(navigation)


if __name__ == "__main__":
    main()
