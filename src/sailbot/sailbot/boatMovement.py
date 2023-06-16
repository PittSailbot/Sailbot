"""
Controls the boat's sails and rudder
"""
import logging
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

from src.sailbot.sailbot import constants as c
from src.sailbot.sailbot.utils.utils import singleton
from src.sailbot.sailbot.utils import boatMath
from src.sailbot.sailbot.peripherals.compass import compass
from src.sailbot.sailbot.peripherals.windvane import windVane
from src.sailbot.sailbot.peripherals.GPS import gps


@singleton
class Sail(Node):
    """
    Drivers and interface for sail

    Attributes:
        angle (float): the current angle of the sail (between 0 and 90 degrees)

        ignore_limits (bool): whether to limit angles outside of range

    Functions:
        auto_adjust(): moves the sail to the optimal angle for speed

        reset(): returns sail to home (fully in)

        set_home(): sets the sail's current position as 0 for calibration
    """

    MIN_ANGLE = int(c.config["SAIL"]["min_angle"])
    MAX_ANGLE = int(c.config["SAIL"]["max_angle"])
    DEFAULT_ANGLE = int(c.config["SAIL"]["default_angle"])

    def __init__(self, ignore_limits=False):
        super().__init__("Sail")
        self.logging = self.get_logger()

        self.logging.info("Initializing sail")

        self.odrive = Odrive(preset="sail")
        self.ignore_limits = ignore_limits

        self._angle = self.DEFAULT_ANGLE

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, angle):
        if not self.ignore_limits:
            if angle < self.MIN_ANGLE:
                angle = self.MIN_ANGLE
            elif angle > self.MAX_ANGLE:
                angle = self.MAX_ANGLE

        self.logging.debug(f"Moving sail to {angle} degrees")
        rotations = self.odrive.rotations_per_degree * (angle - self.angle)
        self.odrive.pos = rotations
        self._angle = angle

    def auto_adjust(self):
        windvane = windvane.windvane()
        wind_direction = windvane.angle

        if wind_direction > 180:
            wind_direction = 180 - (wind_direction - 180)
        self.angle = max(min(wind_direction / 2, 90), 3)

    def reset(self):
        self.angle = self.DEFAULT_ANGLE

    def set_home(self):
        # TODO: Offset odrive or set driver pos to 0?
        # self.odrive.offset = self.odrive.pos
        self.angle = 0


@singleton
class Rudder(Node):
    """
    Drivers and interface for rudder

    Attributes:
        angle (float): the current angle of the rudder (between -45 and 45 deg)

        ignore_limits (bool): whether to limit angles outside of range

    Functions:
        reset(): returns rudder to home (center)

        set_home(): sets the rudder's current position as 0 for calibration
    """

    MIN_ANGLE = int(c.config["RUDDER"]["min_angle"])
    MAX_ANGLE = int(c.config["RUDDER"]["max_angle"])
    DEFAULT_ANGLE = int(c.config["RUDDER"]["default_angle"])

    def __init__(self, ignore_limits=False):
        super().__init__("Rudder")
        self.logging = self.get_logger()

        self.logging.info("Initializing rudder")

        self.odrive = Odrive(preset="rudder")
        self.ignore_limits = ignore_limits

        self._angle = self.DEFAULT_ANGLE

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, angle):
        if not self.ignore_limits:
            if angle < self.MIN_ANGLE:
                angle = self.MIN_ANGLE
            elif angle > self.MAX_ANGLE:
                angle = self.MAX_ANGLE

        self.logging.debug(f"Moving rudder to {angle} degrees")
        rotations = self.odrive.rotations_per_degree * (angle - self.angle)
        self.odrive.pos = rotations
        self._angle = angle

    def reset(self):
        self.angle = self.DEFAULT_ANGLE

    def set_home(self):
        # TODO: Offset odrive or set driver pos to 0?
        # self.odrive.offset = self.odrive.pos
        self.angle = 0


class FailedTurn(RuntimeError):
    """Signals that the boat failed to turn to the proper angle, usually indicates a failed tack"""

    pass


# TODO: Update logic
def turn_to_angle(angle, wait_until_finished=False, allow_tacking=True):
    # TODO: ROS Integration
    """
    Turns the boat using the rudder until it is facing the specified compass angle

    Args:
        angle (float): the angle to turn to
        wait_until_finished (bool): whether to wait until the boat has reached the angle
            # TODO: when false, the rudder won't be reset after the angle is reached, NEEDS TO SPAWN A THREAD
                that thread must also be deleted if another turn_to_angle is called before angle is reached
        allow_tacking (bool): whether the boat is allowed to turn through the 'no go zone'

    Raises:
        - FailedTurn (Exception): the boat failed to turn to the specified angle for whatever reason
    """
    rudder = Rudder()
    compass = compass.compass()
    windvane = windvane.windvane()

    acceptable_error = float(c.config["RUDDER"]["acceptable_error"])
    smoothing_constant = float(c.config["RUDDER"]["smooth_const"])

    logging.info(f"Turning boat from {compass.angle} degrees to {angle} degrees")

    while abs(compass.angle - angle) > acceptable_error and wait_until_finished:
        if ((compass.angle - angle) % 360) < 180:
            # Turn right
            rudder.angle = smoothing_constant * abs(compass.angle - angle)
        else:
            # Turn left
            rudder.angle = -smoothing_constant * abs(compass.angle - angle)

    logging.info(f"Finished turning boat to {angle} degrees")


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

    gps = gps.gps()
    compass = compass.compass()
    windvane = windvane.windvane()
    # rudder = Rudder()

    # acceptable_error = float(c.config['CONSTANTS']['reachedGPSThreshhold'])

    # determine angle we need to turn
    deltaAngle = boatMath.angleToPoint(compass.angle, gps.latitude, gps.longitude, waypoint.lat, waypoint.lon)
    target_angle = (compass.angle + deltaAngle) % 360
    windAngle = windvane.angle

    if (deltaAngle + windAngle) % 360 < windvane.noGoMin:
        target_angle = windvane.noGoMin
    elif (deltaAngle + windAngle) % 360 < windvane.noGoMax:
        target_angle = windvane.noGoMax

    turn_to_angle(target_angle)

    # TODO: create thread to wait until at point


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/drivers"
    rclpy.init(args=args)

    sail = Sail()
    rudder = Rudder()

    try:
        rclpy.spin(sail)
        rclpy.spin(rudder)
    except Exception as e:
        logging.error(f"Exception raised in driver {e}")

    rclpy.shutdown()


if __name__ == "__main__":
    # manually control motors with commands 'sail {value}' and 'rudder {value}'
    sail = Sail()
    rudder = Rudder()

    main()

    while True:
        string = input("  > Enter Input:")

        if string == "quit":
            break

        arr = string.split(" ")

        if arr[0] == "sail" or arr[0] == "s":
            sail.angle = int(arr[1])

        elif arr[0] == "rudder" or arr[0] == "r":
            rudder.angle = int(arr[1])
