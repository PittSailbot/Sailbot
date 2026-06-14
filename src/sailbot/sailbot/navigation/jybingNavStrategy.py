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
    
    # hysteresis for deciding when to jibe
    MIN_JIBE_IMPROVEMENT = 15
    JIBE_MIN_TICKS = 5
    
    # threshold for stern crossing downwind for jibe
    STERN_DOWNWIND_THRESHOLD = 20
    
    # main sail servo controls
    PRE_JIBE_SAIL = 70 # sheet in before jibe
    POST_JIBE_SAIL = 100 # let sails out after
    
    # jib sail servo controls
    PORT_JIB = 0
    CENTER_JIB = 50
    STARBOARD_JIB = 100

    def __init__(self):
        super().__init__()

        # Jibe attributes
        self.jibe_direction = None
        self.is_jibing = False
        self.phase = JibePhase.IDLE
        self.jibe_target_heading = None
        self.prev_wind_angle = None
        self.jibe_ticks = 0

        # sailing parameters
        # valid reach angles (degrees off wind)
        self.reach_angles = range(90, 151, 10)
        self.turn_slow_factor = 0.6
        self.rudder_mult = 0.33

    # =========================================================

    def tick(self):
        # always keep sails optimized when not jibing
        if not self.is_jibing:
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
        downwind = (self.boat_heading + 180) % 360
        
        # compute best reach heading based on wind and target
        desired_heading, desired_direction, desired_reach_angle = self.best_reach_heading(downwind, target_angle)
        
        # correct intial jibe direction
        if self.jibe_direction is None:
            self.jibe_direction = desired_direction

        # decide when to jibe
        # compute potential improvement if taking a jibe
        current_heading = self.reach_heading(downwind, self.jibe_direction, desired_reach_angle)
        current_error = boatMath.degrees_between(current_heading, target_angle)
        new_error = boatMath.degrees_between(desired_heading, target_angle)

        improvement = current_error - new_error
        
        if desired_direction != self.jibe_direction and not self.is_jibing:
            # hysteresis logic for jibing
            if improvement > self.MIN_JIBE_IMPROVEMENT:
                self.jibe_ticks += 1
            else:
                self.jibe_ticks = 0
            
            if self.jibe_ticks >= self.JIBE_MIN_TICKS:
                self.jibe_target_heading = self.reach_heading(
                    downwind,
                    desired_direction,
                    desired_reach_angle
                )
                self.logging.info("Triggering jibe (crossing wind with stern)")
                self.is_jibing = True
                self.phase = JibePhase.PREPARE
                self.jibe_ticks = 0
                return

        # if currently jibing, keep jibing
        if self.is_jibing:
            self.complete_jibe()
            return

        # otherwise continue normally
        self.turn_to_angle(desired_heading)

    # =========================================================

    def turn_to_angle(self, target_angle: int | float | None):
        """
        Sets rudder to turn toward target heading
        """
        
        if target_angle is None:
            return
        
        # calculate signed error between heading and target_angle
        heading_error = (target_angle - self.boat_heading + 540) % 360 - 180

        # only turn if error is higher than acceptable error
        if abs(heading_error) < self.ACCEPTABLE_ERROR:
            self.cmd_rudder_pub.publish(Float32(data=self.RUDDER_CENTER))
            return
        
        # smooth proportional turn
        rudder_offset = heading_error * self.turn_slow_factor * self.rudder_mult

        # ensure command is within the servo limits
        rudder_command = self.RUDDER_CENTER + rudder_offset
        rudder_command = max(
            min(rudder_command, self.RUDDER_MAX),
            self.RUDDER_MIN
        )

        self.cmd_rudder_pub.publish(Float32(data=rudder_command))

    # =========================================================
    
    def best_reach_heading(self, downwind: int | float, target_angle: int | float) -> tuple[int | float, JibeDirection, int | float]:
        best_heading = 180
        best_direction = JibeDirection.JIBING_STARBOARD
        best_reach_angle = 140
        best_error = float("inf")
        for reach_angle in self.reach_angles:
            candidates = [
                (
                    self.reach_heading(
                        downwind,
                        JibeDirection.JIBING_PORT,
                        reach_angle
                    ),
                    JibeDirection.JIBING_PORT
                ),
                (
                    self.reach_heading(
                        downwind,
                        JibeDirection.JIBING_STARBOARD,
                        reach_angle
                    ),
                    JibeDirection.JIBING_STARBOARD
                )
            ]

            for heading, direction in candidates:

                error = boatMath.degrees_between(
                    heading,
                    target_angle
                )

                if error < best_error:
                    best_error = error
                    best_heading = heading
                    best_direction = direction
                    best_reach_angle = reach_angle
        return best_heading, best_direction, best_reach_angle
    
    # =========================================================

    def reach_heading(self, downwind: int | float, direction: JibeDirection, reach_angle: int | float) -> int | float:
        """ Find the reach heading given wind and jibe direction """
        return (
            (downwind - reach_angle) % 360
            if direction == JibeDirection.JIBING_STARBOARD
            else (downwind + reach_angle) % 360
        )

    # =========================================================

    def complete_jibe(self):
        """
        Controlled jibe sequence:
        PREPARE → TURN → CROSS → RECOVER
        """

        # prepare to jibe
        if self.phase == JibePhase.PREPARE:
            self.cmd_sail_pub.publish(Float32(data=self.PRE_JIBE_SAIL))
            self.cmd_jib_pub.publish(Float32(
                data = self.STARBOARD_JIB
                if self.jibe_direction == JibeDirection.JIBING_STARBOARD
                else self.PORT_JIB
            ))
            self.phase = JibePhase.TURN
            self.logging.info("Jibe: PREPARE → TURN")
            return

        # turn to rotate through wind
        if self.phase == JibePhase.TURN:

            # compute downwind direction
            downwind = (self.boat_heading + 180) % 360

            # turn to jibe target heading from start of jibe
            self.turn_to_angle(self.jibe_target_heading)

            # Near dead downwind → next phase
            heading_diff = boatMath.degrees_between(self.boat_heading, downwind)
            if heading_diff < 20:
                self.prev_wind_angle = self.wind_angle
                self.phase = JibePhase.CROSS
                self.logging.info("Jibe: TURN → CROSS")

            return

        # cross the wind and detect wind flip
        if self.phase == JibePhase.CROSS:
            # move jib in
            self.cmd_jib_pub.publish(Float32(data=self.CENTER_JIB))

            # continue heading from before to complete the crossing
            self.turn_to_angle(self.jibe_target_heading)
            
            # find downwind
            downwind = (self.boat_heading + 180) % 360

            wind_crossed = False # check if crossed dead downwind
            stern_crossed = False # check if stern crossed within threshold of downwind
            
            # detect wind flip (stern passes through wind)
            if self.prev_wind_angle is not None:
                # ignore noisy region using deadband
                if not self.DEADBAND_LOW < self.wind_angle < self.DEADBAND_HIGH:
                    # detect crossing in either direction if past a certain range
                    # compute signed change in wind angle (wrap-safe)
                    delta = (self.wind_angle - self.prev_wind_angle + 540) % 360 - 180

                    # detect flip using a crossing 180° boundary OR a large jump (wind vane lags then snaps to wind causing large delta)
                    flipped_sign = (self.prev_wind_angle - 180) * (self.wind_angle - 180) < 0
                    large_jump = abs(delta) > 90

                    wind_crossed = flipped_sign or large_jump

            self.prev_wind_angle = self.wind_angle
            
            # detect stern crossing
            heading_diff = boatMath.degrees_between(self.boat_heading, downwind)
            stern_crossed = heading_diff < self.STERN_DOWNWIND_THRESHOLD

            if wind_crossed and stern_crossed:
                self.phase = JibePhase.RECOVER
                self.logging.info("Jibe: CROSS → RECOVER")

            return

        # recover and stabilize after jibing
        if self.phase == JibePhase.RECOVER:

            self.cmd_sail_pub.publish(Float32(data=self.POST_JIBE_SAIL))
            self.cmd_rudder_pub.publish(Float32(data=self.RUDDER_CENTER))
            
            self.jibe_direction = (
                JibeDirection.JIBING_PORT
                if self.jibe_direction == JibeDirection.JIBING_STARBOARD
                else JibeDirection.JIBING_STARBOARD
            )

            self.cmd_jib_pub.publish(Float32(
                data = self.STARBOARD_JIB
                if self.jibe_direction == JibeDirection.JIBING_STARBOARD
                else self.PORT_JIB
            ))

            self.logging.info("Jibe complete")

            self.is_jibing = False
            self.phase = JibePhase.IDLE
            self.prev_wind_angle = None
            self.jibe_target_heading = None
            self.jibe_ticks = 0

            return


# main for this python module
def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ.get("ROS_LOG_DIR_BASE", "/tmp") + "/navigation"
    rclpy.init(args=args)

    navigation = JibingNavigation()
    rclpy.spin(navigation)

if __name__ == "__main__":
    main()