"""
Handles autonomous navigation to a desired GPS point using a Jibing strategy
"""

import os
from enum import Enum

import rclpy
from std_msgs.msg import Float32, String

from sailbot import constants as c
from sailbot.utils import boatMath
from sailbot.utils.utils import Waypoint

from sailbot.navigation.navStrategy import NavigationStrategy

# Enums for use in jibing navigation strategy

# which direction to jibe (zig-zag)
class JibeDirection(Enum):
    JIBING_STARBOARD = True
    JIBING_PORT = False

# states for a jibe
class JibePhase(Enum):
    IDLE = 0
    PREPARE = 1
    TURN = 2
    CROSS = 3
    RECOVER = 4

# navigation strategy
class JibingNavigation(NavigationStrategy):
    """Autonomously navigates using downwind zig-zag + controlled jibing"""

    # CONSTANTS
    # crossing angles to consider the boat to have crossed the wind
    LOW_CROSSING_ANGLE = 160
    HIGH_CROSSING_ANGLE = 200
    
    # deadband to ignore potential noisy wind data
    DEADBAND_LOW = 170
    DEADBAND_HIGH = 190

    def __init__(self):
        super().__init__()

        # Jibe attributes
        self.jibe_direction = JibeDirection.JIBING_STARBOARD
        self.is_jibing = False
        self.phase = JibePhase.IDLE
        self.prev_wind_angle = None

        # sailing parameters
        self.reach_angle = 140  # degrees off wind (broad reach)
        self.turn_slow_factor = 0.6
        self.rudder_mult = 0.33

        self.pre_jibe_sail = 0.7   # sheet in before jibe
        self.post_jibe_sail = 1.0  # let sails out after

    # =========================================================

    def tick(self):
        # always keep sails optimized
        self.auto_adjust_sail()

        if self.wp.target_waypoint is not None:
            self.go_to_gps(self.wp.target_waypoint)

    # =========================================================

    def go_to_gps(self, target: Waypoint):
        """
        Moves the boat to the target GPS using:
        - Broad reach heading
        - Controlled jibing when switching sides
        """

        # Direction to waypoint
        target_angle = boatMath.angle_to_point(
            self.boat_position.lat,
            self.boat_position.lon,
            target.lat,
            target.lon
        )

        # compute downwind direction
        downwind = (self.wind_angle + self.boat_heading + 180) % 360

        # compute broad reach heading based on jibe direction
        desired_heading = self.reach_heading(downwind, self.jibe_direction)

        # get 
        angle_to_target = boatMath.degrees_between(desired_heading, target_angle)

        # decide when to jibe
        if angle_to_target > 60 and not self.is_jibing:
            self.logging.info("Triggering jibe (crossing wind with stern)")
            self.is_jibing = True
            self.phase = JibePhase.PREPARE
            return

        # if currently jibing, keep jibing
        if self.is_jibing:
            self.complete_jibe()
            return

        # otherwise continue normally
        self.turn_to_angle(desired_heading)

    # =========================================================

    def turn_to_angle(self, target_angle: int | float):
        """
        Sets rudder to turn toward target heading
        """

        if boatMath.is_within_angle(
            target_angle,
            (self.boat_heading - self.ACCEPTABLE_ERROR) % 360,
            (self.boat_heading + self.ACCEPTABLE_ERROR) % 360
        ):
            self.cmd_rudder_pub.publish(Float32(data=self.RUDDER_CENTER))
            return

        rudder_angle = abs(self.boat_heading - target_angle)
        
        # not turning left, so turn right
        if not (self.boat_heading - target_angle) % 360 < 180:
            rudder_angle *= -1

        # smooths out turning
        rudder_angle *= self.turn_slow_factor

        rudder_angle = min(rudder_angle, self.RUDDER_MAX)
        rudder_angle = max(rudder_angle, self.RUDDER_MIN)

        self.cmd_rudder_pub.publish(Float32(data=rudder_angle * self.rudder_mult))

    # =========================================================
    
    def reach_heading(self, downwind: int | float, direction: JibeDirection) -> int | float:
        """ Find the broad reach heading given wind and jibe direction """
        return (
            (downwind - self.reach_angle) % 360
            if direction == JibeDirection.JIBING_STARBOARD
            else (downwind + self.reach_angle) % 360
        )

    # =========================================================

    def complete_jibe(self):
        """
        Controlled jibe sequence:
        PREPARE → TURN → CROSS → RECOVER
        """

        # prepare to jibe
        if self.phase == JibePhase.PREPARE:
            self.cmd_sail_pub.publish(Float32(data=self.pre_jibe_sail))
            self.cmd_jib_pub.publish(Float32(data=self.pre_jibe_sail))
            self.phase = JibePhase.TURN
            self.logging.info("Jibe: PREPARE → TURN")
            return

        # turn to rotate through wind
        if self.phase == JibePhase.TURN:

            # compute downwind direction
            downwind = (self.wind_angle + self.boat_heading + 180) % 360

            # compute broad reach headings and choose correct (opposite) target using jibe direction
            # rotate toward the opposite reach (the new tack)
            opposite = (
                JibeDirection.JIBING_PORT
                if self.jibe_direction == JibeDirection.JIBING_STARBOARD
                else JibeDirection.JIBING_STARBOARD
            )
            target_dir = self.reach_heading(downwind, opposite)
            self.turn_to_angle(target_dir)

            # Near dead downwind → next phase
            heading_diff = boatMath.degrees_between(self.boat_heading, downwind)
            if heading_diff < 20:
                self.phase = JibePhase.CROSS
                self.logging.info("Jibe: TURN → CROSS")

            return

        # cross the wind and detect wind flip
        if self.phase == JibePhase.CROSS:
            # stabilize and wait for crossing
            self.cmd_jib_pub.publish(Float32(data=0.0))
            self.cmd_rudder_pub.publish(Float32(data=self.RUDDER_CENTER))

            crossed = False
            
            # detect wind flip (stern passes through wind)
            if self.prev_wind_angle is not None:
                # ignore noisy region using deadband
                if not self.DEADBAND_LOW < self.wind_angle < self.DEADBAND_HIGH:
                    # detect a crossing if past certain range
                    crossed = self.prev_wind_angle < self.LOW_CROSSING_ANGLE and self.wind_angle > self.HIGH_CROSSING_ANGLE

            self.prev_wind_angle = self.wind_angle

            if crossed:
                self.phase = JibePhase.RECOVER
                self.logging.info("Jibe: CROSS → RECOVER")

            return

        # recover and stabilize after jibing
        if self.phase == JibePhase.RECOVER:

            self.cmd_sail_pub.publish(Float32(data=self.post_jibe_sail))
            self.cmd_rudder_pub.publish(Float32(data=self.RUDDER_CENTER))
            
            self.jibe_direction = (
                JibeDirection.JIBING_PORT
                if self.jibe_direction == JibeDirection.JIBING_STARBOARD
                else JibeDirection.JIBING_STARBOARD
            )

            jib_trim = (
                self.post_jibe_sail
                if self.jibe_direction == JibeDirection.JIBING_STARBOARD
                else -self.post_jibe_sail
            )
            self.cmd_jib_pub.publish(Float32(data=jib_trim))

            self.logging.info("Jibe complete")

            self.is_jibing = False
            self.phase = JibePhase.IDLE
            self.prev_wind_angle = None
            
            # compute downwind and new reach heading based on direction
            downwind = (self.wind_angle + self.boat_heading + 180) % 360
            desired_heading = self.reach_heading(downwind, self.jibe_direction)

            self.turn_to_angle(desired_heading)

            return


# main for this python module
def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ.get("ROS_LOG_DIR_BASE", "/tmp") + "/navigation"
    rclpy.init(args=args)

    navigation = JibingNavigation()
    rclpy.spin(navigation)

if __name__ == "__main__":
    main()