"""
Handles autonomous navigation to a desired GPS point using a Jibing strategy
"""

import os
from enum import Enum

import rclpy
from std_msgs.msg import Float32, String

from sailbot import constants as c
from sailbot.utils import boatMath

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

    def __init__(self):
        super().__init__()

        # Jibe attributes
        self.jibe_direction = JibeDirection.JIBING_STARBOARD
        self.jibing = False
        self.phase = JibePhase.IDLE
        self.prev_wind_angle = None

        # sailing parameters
        self.reach_angle = 140  # degrees off wind (broad reach)
        self.turn_slow_factor = 0.6
        self.rudder_mult = 0.33

        self.pre_jibe_sail = 0.7   # sheet in before jibe
        self.post_jibe_sail = 1.0  # let sails out after
        
        # crossing angles to consider the boat to have crossed the wind
        self.LOW_CROSSING_ANGLE = 160
        self.HIGH_CROSSING_ANGLE = 200
        
        # deadband to ignore potential noisy wind data
        self.DEADBAND_LOW = 170
        self.DEADBAND_HIGH = 190

        self.latest_waypoint = None

    # =========================================================

    def tick(self):
        """Update Navigation decision-making, setting sail/jib/rudder"""

        # No waypoint → do nothing
        if self.latest_waypoint is None:
            return

        target = self.latest_waypoint

        # Check if reached waypoint
        if boatMath.distance_between(self.boat_position, target) < float(c.config["CONSTANTS"]["reached_waypoint_distance"]):
            self.logging.info(f"Reached {target}. Heaving.")

            self.cmd_sail_pub.publish(Float32(data=0.0))
            self.cmd_jib_pub.publish(Float32(data=0.0))
            self.cmd_rudder_pub.publish(Float32(data=0.0))
            return

        # always keep sails optimized
        self.auto_adjust_sail()

        self.go_to_gps(target)

    # =========================================================

    def go_to_gps(self, target):
        """
        Moves the boat to the target GPS using:
        - Broad reach zig-zag
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

        # decide when to jibe
        angle_to_target = boatMath.degrees_between(desired_heading, target_angle)

        if angle_to_target > 60 and not self.jibing:
            self.logging.info("Triggering jibe (crossing wind with stern)")
            self.jibing = True
            self.phase = JibePhase.PREPARE
            return

        # if currently jibing, keep jibing
        if self.jibing:
            self.complete_jibe()
            return

        # otherwise continue normally
        self.turn_to_angle(desired_heading)

    # =========================================================

    def turn_to_angle(self, target_angle):
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

        turning_left = (self.boat_heading - target_angle) % 360 < 180

        if turning_left:
            rudder_angle = abs(self.boat_heading - target_angle)
        else:
            rudder_angle = -abs(self.boat_heading - target_angle)

        # smooths out turning
        rudder_angle *= self.turn_slow_factor

        rudder_angle = min(rudder_angle, self.RUDDER_MAX)
        rudder_angle = max(rudder_angle, self.RUDDER_MIN)

        self.cmd_rudder_pub.publish(Float32(data=rudder_angle * self.rudder_mult))

    # =========================================================
    
    def reach_heading(self, downwind, direction):
        if direction == JibeDirection.JIBING_STARBOARD:
            return (downwind - self.reach_angle) % 360
        else:
            return (downwind + self.reach_angle) % 360

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

            self.jibing = False
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