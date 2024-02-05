"""
Controls the boat's sails and rudder. Also includes functions to autonomously drive the boat to a specific gps position or heading
"""
import logging
import os

import rclpy
from rclpy.node import Node

from sailbot.peripherals.GPS import GPS
from sailbot.peripherals.windvane import WindVane, NoGoZone
from sailbot.peripherals.compass import Compass
from sailbot import constants as c
from sailbot.peripherals.Odrive import Odrive
from sailbot.utils import boatMath


# TODO: Add ROS Callback
@singleton
class Sail(Node):
    """
    Drivers and interface for sail

    Attributes:
        angle (float): the current angle of the sail (between 0 and 90 degrees)

    Functions:
        auto_adjust(): moves the sail to the optimal angle for speed

        reset(): returns sail to home (fully in)

        set_home(): sets the sail's current position as 0 for calibration
    """

    MIN_ANGLE = int(c.config["SAIL"]["min_angle"])
    MAX_ANGLE = int(c.config["SAIL"]["max_angle"])
    DEFAULT_ANGLE = int(c.config["SAIL"]["default_angle"])

    def __init__(self):
        super().__init__("Sail")
        self.logging = self.get_logger()

        self.logging.info("Initializing sail")

        self.odrive = Odrive(preset="sail")

        self._angle = self.DEFAULT_ANGLE

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, angle):
        if angle < self.MIN_ANGLE:
            angle = self.MIN_ANGLE
        elif angle > self.MAX_ANGLE:
            angle = self.MAX_ANGLE

        self.logging.debug(f"Moving sail to {angle} degrees")
        rotations = map(angle, self.MIN_ANGLE, self.MAX_ANGLE, 0, self.odrive.max_rotations)
        self.odrive.pos = rotations
        self._angle = angle

    def auto_adjust(self):
        wind_direction = WindVane().angle

        if wind_direction > 180:
            wind_direction = 180 - (wind_direction - 180)
        self.angle = max(min(wind_direction / 2, 90), 3)

    def reset(self):
        self.angle = self.DEFAULT_ANGLE

    def set_home(self):
        # TODO: Offset odrive or set driver pos to 0?
        # self.odrive.offset = self.odrive.pos
        self._angle = 0


# TODO: Add ROS Callback
@singleton
class Rudder(Node):
    """
    Drivers and interface for rudder

    Attributes:
        angle (float): the current angle of the rudder (between -45 and 45 deg)

    Functions:
        reset(): returns rudder to home (center)

        set_home(): sets the rudder's current position as 0 for calibration
    """

    MIN_ANGLE = int(c.config["RUDDER"]["min_angle"])
    MAX_ANGLE = int(c.config["RUDDER"]["max_angle"])
    DEFAULT_ANGLE = int(c.config["RUDDER"]["default_angle"])

    def __init__(self):
        super().__init__("Rudder")
        self.logging = self.get_logger()

        self.logging.info("Initializing rudder")

        self.odrive = Odrive(preset="rudder")

        self._angle = self.DEFAULT_ANGLE

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, angle):
        if angle < self.MIN_ANGLE:
            angle = self.MIN_ANGLE
        elif angle > self.MAX_ANGLE:
            angle = self.MAX_ANGLE

        self.logging.debug(f"Moving rudder to {angle} degrees")
        rotations = map(angle, self.MIN_ANGLE, self.MAX_ANGLE, -self.odrive.max_rotations / 2, self.odrive.max_rotations / 2)
        self.odrive.pos = rotations
        self._angle = angle

    def reset(self):
        self.angle = self.DEFAULT_ANGLE

    def set_home(self):
        # TODO: Offset odrive or set driver pos to 0?
        # self.odrive.offset = self.odrive.pos
        self._angle = 0


class FailedTurn(RuntimeError):
    """Signals that the boat failed to turn to the proper angle, usually indicates a failed tack"""
    pass


# TODO: Update logic
def turn_to_angle(target_angle, allow_tacking=True, wait_until_finished=False):
    # TODO: ROS Integration
    """
    Turns the boat using the rudder until it is facing the specified compass angle

    Args:
        target_angle (float): the angle to turn to
        allow_tacking (bool): whether the boat is allowed to turn through the 'no go zone'
        wait_until_finished (bool): whether to wait until the boat has reached the angle
            # TODO: when false, the rudder won't be reset after the angle is reached, NEEDS TO SPAWN A THREAD
                that thread must also be deleted if another turn_to_angle is called before angle is reached

    Raises:
        - FailedTurn (Exception): the boat failed to turn to the specified angle for whatever reason
    """
    rudder = Rudder()
    compass = Compass()

    acceptable_error = float(c.config["RUDDER"]["acceptable_error"])
    smoothing_constant = float(c.config["RUDDER"]["smooth_const"])

    # logging.info(f"Turning boat from {compass.angle} degrees to {angle} degrees")
    while True:
        boat_angle = compass.angle

        if abs(boat_angle - target_angle) > acceptable_error:
            logging.info(f"Finished turning boat to {target_angle} degrees")
            break

        if allow_tacking:
            if (boat_angle - target_angle) % 360 < 180:
                rudder.angle = smoothing_constant * abs(boat_angle - target_angle)  # Turn right
            else:
                rudder.angle = -smoothing_constant * abs(boat_angle - target_angle)  # Turn left
        else:
            wind_angle = WindVane().angle
            if ((boat_angle < target_angle) and boat_angle <= wind_angle <= target_angle) or (target_angle < boat_angle and (target_angle >= wind_angle and wind_angle <= boat_angle)):
                rudder.angle = -smoothing_constant * abs(boat_angle - target_angle)  # Turn left
            else:
                rudder.angle = smoothing_constant * abs(boat_angle - target_angle)  # Turn right

        if not wait_until_finished:
            break


# TODO: Update logic
def go_to_gps(waypoint, wait_until_finished=False):
    # TODO: ROS integration
    """
    Moves the boat to the target GPS
    Args:
        waypoint (Waypoint): the GPS point to go to
        wait_until_finished (bool): whether to wait until the boat has reached the GPS point
    Raises:

    """

    gps = GPS()
    compass = Compass()
    windvane = WindVane()
    # rudder = Rudder()

    # acceptable_error = float(c.config['CONSTANTS']['reachedGPSThreshhold'])

    # determine angle we need to turn
    deltaAngle = boatMath.angleToPoint(compass.angle, gps.latitude, gps.longitude, waypoint.lat, waypoint.lon)
    target_angle = (compass.angle + deltaAngle) % 360
    windAngle = windvane.angle

    if (deltaAngle + windAngle) % 360 < windvane.no_go_zone.left_bound:
        target_angle = windvane.no_go_zone.left_bound
    elif (deltaAngle + windAngle) % 360 < windvane.no_go_zone.right_bound:
        target_angle = windvane.no_go_zone.right_bound

    turn_to_angle(target_angle)

    # TODO: create thread to wait until at point


def map(x, min1, max1, min2, max2):
    # converts value x, which ranges from min1-max1, to a corresponding value ranging from min2-max2
    # ex: map(0.3, 0, 1, 0, 100) returns 30
    # ex: map(70, 0, 100, 0, 1) returns .7
    x = min(max(x, min1), max1)
    return min2 + (max2 - min2) * ((x - min1) / (max1 - min1))


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/drivers"
    rclpy.init(args=args)

    sail = Sail()
    rudder = Rudder()

    while True:
        try:
            string = input("> Enter Input: ")

            arr = string.split(" ")

            if arr[0] == "sail" or arr[0] == "s":
                sail.angle = int(arr[1])

            elif arr[0] == "rudder" or arr[0] == "r":
                rudder.angle = int(arr[1])
        except KeyboardInterrupt:
            break

    rclpy.shutdown()


if __name__ == "__main__":
    # manually control motors with commands 'sail {value}' and 'rudder {value}'
    main()


