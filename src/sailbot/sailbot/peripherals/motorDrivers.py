"""
Contains drivers for sail and rudder 
"""

import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

from sailbot import constants as c
from sailbot.peripherals.Odrive import Odrive
from sailbot.utils import boatMath

class MotorDriver(Node):
    """Driver interface for rudder
    Listens to:
        - /cmd_rudder (String): Sets the rudder to the specified angle (within config limits)
        - /cmd_rudder_offset (String): Shifts the default angle
    """

    RUDDER_MIN_ANGLE = int(c.config["RUDDER"]["min_angle"])
    RUDDER_MAX_ANGLE = int(c.config["RUDDER"]["max_angle"])

    SAIL_MIN_ANGLE = int(c.config["SAIL"]["min_angle"])
    SAIL_MAX_ANGLE = int(c.config["SAIL"]["max_angle"])

    def __init__(self):
        super().__init__("motorDriver")
        self.logging = self.get_logger()
        self.logging.info("Initializing motor driver")

        self.rudder_odrive = Odrive(preset="rudder")
        self.rudder_offset = 0
        self.rudder_sub = self.create_subscription(Float32, "cmd_rudder", self.rudder_callback, 10)
        self.rudder_offset_sub = self.create_subscription(Float32, "offset_rudder", self.rudder_offset_callback, 10)

        self.sail_odrive = Odrive(preset="sail")
        self.sail_offset = 0
        self.sail_sub = self.create_subscription(Float32, "cmd_sail", self.sail_callback, 10)
        self.sail_offset_sub = self.create_subscription(Float32, "offset_sail", self.sail_offset_callback, 10)

    def rudder_callback(self, msg):
        angle = float(msg.data)

        angle = max(angle, self.RUDDER_MIN_ANGLE)
        angle = min(angle, self.RUDDER_MAX_ANGLE)

        self.logging.debug(f"Moving rudder to {angle} degrees")
        rotations = boatMath.remap(angle + self.rudder_offset, self.RUDDER_MIN_ANGLE, self.RUDDER_MAX_ANGLE, -self.rudder_odrive.max_rotations / 2, self.rudder_odrive.max_rotations / 2)
        self.rudder_odrive.pos = rotations

    def rudder_offset_callback(self, msg):
        messageVal = float(msg.data)
        
        self.rudder_offset += messageVal
        self.logging.debug(f"Changing Rudder offset by: {messageVal}%")

    def sail_callback(self, msg):
        angle = float(msg.data)

        angle = max(angle, self.RUDDER_MIN_ANGLE)
        angle = min(angle, self.RUDDER_MAX_ANGLE)

        self.logging.debug(f"Moving sail to {angle} degrees")
        rotations = boatMath.remap(angle + self.sail_offset, self.SAIL_MIN_ANGLE, self.SAIL_MAX_ANGLE, -self.sail_odrive.max_rotations / 2, self.sail_odrive.max_rotations / 2)
        self.sail_odrive.pos = rotations

    def sail_offset_callback(self, msg):
        messageVal = float(msg.data)
        
        self.sail_offset += messageVal
        self.logging.debug(f"Changing Sail offset by: {messageVal}%")

def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/motorDrivers"
    rclpy.init(args=args)

    motorDriver = MotorDriver()
    rclpy.spin(motorDriver)