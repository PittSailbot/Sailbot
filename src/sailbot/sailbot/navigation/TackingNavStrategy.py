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
        - /gps (String):
        - /compass (String):
        - /windvane (String):
        - /control_state (Bool):
    Publishes to:
        - /cmd_rudder (String)
        - /cmd_sail (String)
    """

    SENSOR_TIMEOUT = float(c.config["NAVIGATION"]["SENSOR_TIMEOUT"])

    def __init__(self):
        super().__init__()
        self.aborted_tacks = 0

        self.allow_tacking = True
        self.mode = None
        self.emergency_tack = False

        self.rudderMult = 0.33

        self.jibe_angle_increase = 75  # (degrees)

    def tick(self):
        """Update Navigation decision-making, setting sail/jib/rudder"""
        now = self.get_clock().now()
        
        gps_stale = (now - self.last_gps_time).nanoseconds / 1e9 > SENSOR_TIMEOUT
        imu_stale = (now - self.last_imu_time).nanoseconds / 1e9 > SENSOR_TIMEOUT
        wind_stale = (now - self.last_wind_time).nanoseconds / 1e9 > SENSOR_TIMEOUT

        if self.boat_position is None or self.boat_heading is None or self.wind_angle is None or gps_stale or imu_stale or wind_stale:
            if self.boat_position is None or gps_stale:
                self.logging.warning("No fresh GPS for autonomy to function")
            if self.boat_heading is None or imu_stale:
                self.logging.warning("No fresh IMU data for autonomy to function")
            if self.wind_angle is None or wind_stale:
                self.logging.warning("No fresh windvane for autonomy to function")
            self.heave_to()
            return

        if self.wp.target_waypoint is None:
            self.heave_to()
            return
            
        self.status = f"{self.wp}"
        self.go_to_gps(self.target_waypoint)
        self.logging.info(self.status)

    def go_to_gps(self, target):
        """
        Moves the boat to the target GPS
        Args:
            target (Waypoint): the GPS point to go to
        """
        self.status += f"\nNavigating to {target}:"
        
        target_angle = boatMath.angle_to_point(self.boat_position.lat, self.boat_position.lon, target.lat, target.lon)

        start_wp = None
        if hasattr(self.wp, 'current_waypoint_index') and self.wp.current_waypoint_index > 0:
            start_wp = self.wp.waypoints[self.wp.current_waypoint_index - 1]

        # Cross track error compensation
        if start_wp is not None:
            xte = boatMath.cross_track_error(start_wp, target, self.boat_position)
            # Simple proportional XTE correction to compensate for leeway
            xte_gain = float(c.config.get("NAVIGATION", {}).get("xte_gain", 5.0))
            max_xte_correction = float(c.config.get("NAVIGATION", {}).get("max_xte_correction", 45.0))
            correction = max(-max_xte_correction, min(max_xte_correction, xte * xte_gain))
            
            # Apply XTE correction to target_angle (positive XTE -> steer left -> decrease angle)
            target_angle = (target_angle - correction) % 360

        delta_angle = (target_angle - self.boat_heading) % 360
        if delta_angle > 180:
            delta_angle -= 360

        # Boat can't sail straight upwind; snap angle to the closest allowed no_go_zone bound if target angle is in irons
        if self.mode is None and boatMath.is_within_angle(self.boat_heading, self.no_go_zone_left_bound, self.no_go_zone_right_bound):
            self.logging.warning("Unexpectedly in irons, turning to nearest edge")
            self.emergency_tack = True
            if boatMath.degrees_between(target_angle, self.no_go_zone_left_bound) < boatMath.degrees_between(target_angle, self.no_go_zone_right_bound):
                self.mode = Mode.TACKING_PORT
            else:
                self.mode = Mode.TACKING_STARBOARD

        elif self.mode is None and boatMath.is_within_angle(target_angle, self.no_go_zone_left_bound, self.no_go_zone_right_bound):
            # Target is in no-go zone, implement cross-track corridor hysteresis (laylines)
            cross_track_corridor = float(c.config.get("NAVIGATION", {}).get("cross_track_corridor", 20.0))
            
            if not hasattr(self, 'current_upwind_tack') or self.current_upwind_tack is None:
                if boatMath.degrees_between(self.boat_heading, self.no_go_zone_left_bound) < boatMath.degrees_between(self.boat_heading, self.no_go_zone_right_bound):
                    self.current_upwind_tack = Mode.TACKING_PORT
                else:
                    self.current_upwind_tack = Mode.TACKING_STARBOARD

            if start_wp is not None:
                if xte > cross_track_corridor and self.current_upwind_tack == Mode.TACKING_STARBOARD:
                    self.current_upwind_tack = Mode.TACKING_PORT
                    self.logging.info("Cross-track corridor exceeded to the right. Switching to PORT tack.")
                elif xte < -cross_track_corridor and self.current_upwind_tack == Mode.TACKING_PORT:
                    self.current_upwind_tack = Mode.TACKING_STARBOARD
                    self.logging.info("Cross-track corridor exceeded to the left. Switching to STARBOARD tack.")

            if self.current_upwind_tack == Mode.TACKING_PORT:
                target_angle = self.no_go_zone_left_bound
            else:
                target_angle = self.no_go_zone_right_bound

            self.logging.info(f"Need to go upwind, setting target angle to {target_angle} on tack {self.current_upwind_tack}")
        else:
            self.current_upwind_tack = None

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
        if boatMath.is_within_angle(target_angle, (self.boat_heading - self.ACCEPTABLE_ERROR) % 360, (self.boat_heading + self.ACCEPTABLE_ERROR) % 360):
            self.status += f"\n  Holding boat at {target_angle} degrees")
            msg = Float32()
            msg.data = self.RUDDER_CENTER
            self.cmd_rudder_pub.publish(msg)
            return

        no_go_zone_center = self.wind_angle + self.boat_heading
        turning_left = (self.boat_heading - target_angle) % 360 < 180
        is_no_go_zone_left = (self.wind_angle - 180) > 0
        no_go_distance = boatMath.degrees_between(self.boat_heading, no_go_zone_center)
        if not self.allow_tacking:
            no_go_distance += self.jibe_angle_increase
        distance_to_target = boatMath.degrees_between(self.boat_heading, target_angle)

        need_to_tack = not turning_left and not is_no_go_zone_left and distance_to_target > no_go_distance
        need_to_tack = need_to_tack or (turning_left and is_no_go_zone_left and distance_to_target > no_go_distance)

        if need_to_tack:
            # Shortest path to target is across the no-go-zone
            self.mode = Mode.TACKING_STARBOARD if turning_left else Mode.TACKING_PORT
            tack_or_jibe = "Tack" if self.allow_tacking else "Jibe"
            self.logging.info(f"Starting {tack_or_jibe}: {self.mode}")
            return self.complete_tack()

        no_go_zone_left_bound, self.no_go_zone_right_bound = boatMath.get_no_go_zone_bounds(self.wind_angle, self.boat_heading)
        if boatMath.degrees_between(self.boat_heading, no_go_zone_left_bound) < boatMath.degrees_between(self.boat_heading, self.no_go_zone_right_bound):
            target_angle = no_go_zone_left_bound
        else:
            target_angle = self.no_go_zone_right_bound

        if turning_left:
            self.status += f"\n  Turning left from {self.boat_heading} degrees to {target_angle} degrees"
            rudder_angle = self.SMOOTHING_CONSTANT * abs(self.boat_heading - target_angle)
        else:
            self.status += f"\n  Turning right from {self.boat_heading} degrees to {target_angle} degrees"
            rudder_angle = -self.SMOOTHING_CONSTANT * abs(self.boat_heading - target_angle)

        rudder_angle = min(rudder_angle, float(c.config["RUDDER"]["max_angle"]))
        rudder_angle = max(rudder_angle, float(c.config["RUDDER"]["min_angle"]))

        msg = Float32()
        msg.data = rudder_angle * self.rudderMult
        self.cmd_rudder_pub.publish(msg)

    def complete_tack(self):
        tack_or_jibe = "Tack" if self.allow_tacking else "Jibe"

        if boatMath.degrees_between(self.boat_heading, self.no_go_zone_left_bound) < boatMath.degrees_between(self.boat_heading, self.no_go_zone_right_bound):
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

        self.status += f"Continuing {tack_or_jibe}"


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/navigation"
    rclpy.init(args=args)

    navigation = TackingNavigationStrategy()
    rclpy.spin(navigation)


if __name__ == "__main__":
    main()
