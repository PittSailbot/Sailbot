"""
WIP file to decouple sails and rudder from main for better testing
Drivers and interface for moving the boat's sails and rudder
"""
import logging

from src.sailbot.sailbot import constants as c
from src.sailbot.sailbot.utils.utils import singleton
from src.sailbot.sailbot.utils import boatMath
from src.sailbot.sailbot.peripherals.compass import compass
from src.sailbot.sailbot.peripherals.windvane import windVane
from src.sailbot.sailbot.peripherals.GPS import gps


@singleton
class Sail:
    MIN_ANGLE = int(c.config["SAIL"]["min_angle"])
    MAX_ANGLE = int(c.config["SAIL"]["max_angle"])
    DEFAULT_ANGLE = int(c.config["SAIL"]["default_angle"])

    def __init__(self):
        self._angle = self.DEFAULT_ANGLE
        logging.info("Initializing sail")

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, angle):
        if angle < self.MIN_ANGLE:
            angle = self.MIN_ANGLE
        elif angle > self.MAX_ANGLE:
            angle = self.MAX_ANGLE
        logging.debug(f"Moving sail to {angle} degrees")
        self._angle = angle

    def reset(self):
        self.angle = self.DEFAULT_ANGLE


@singleton
class Rudder:
    MIN_ANGLE = int(c.config["RUDDER"]["min_angle"])
    MAX_ANGLE = int(c.config["RUDDER"]["max_angle"])
    DEFAULT_ANGLE = int(c.config["RUDDER"]["default_angle"])

    def __init__(self):
        self._angle = self.DEFAULT_ANGLE
        logging.info("Initializing rudder")

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, angle):
        if angle < self.MIN_ANGLE:
            angle = self.MIN_ANGLE
        elif angle > self.MAX_ANGLE:
            angle = self.MAX_ANGLE
        logging.debug(f"Moving rudder to {angle} degrees")
        self._angle = angle

    def reset(self):
        self.angle = self.DEFAULT_ANGLE


def turn_to_angle(angle, wait_until_finished=False):
    # TODO: ROS Integration
    """
    Turns the boat using the rudder until it is facing the specified angle
    Args:
        angle (float): the angle to turn to
        wait_until_finished (bool): whether to wait until the boat has reached the angle
            # TODO: when false, the rudder won't be reset after the angle is reached, NEEDS TO SPAWN A THREAD
                that thread must also be deleted if another turn_to_angle is called before angle is reached
    """
    rudder = Rudder()
    compass = compass.compass()

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


def go_to_gps(waypoint, wait_until_finished=False):
    # TODO: ROS integration
    """
    Moves the boat to the target GPS
    Args:
        waypoint (Waypoint): the GPS point to go to
        wait_until_finished (bool): whether to wait until the boat has reached the GPS point
    """

    gps = gps.gps()
    compass = compass.compass()
    windvane = windvane.windvane()
    #rudder = Rudder()

    #acceptable_error = float(c.config['CONSTANTS']['reachedGPSThreshhold'])

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

    #if eventUtils.distance_between(waypoint, eventUtils.Waypoint(gps.latitude, gps.longitude)) < acceptable_error:
        # if we are very close to GPS coord
        #if cycleTargets:
            #targets.append((lat, long))
        #currentTarget = None
        #rudder.reset()
