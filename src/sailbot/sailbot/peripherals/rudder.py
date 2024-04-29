"""
Contains drivers for sail and rudder and can autonomously sail to a specific gps position or heading
"""
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

from sailbot import constants as c
from sailbot.peripherals.Odrive import Odrive
from sailbot.utils import boatMath


class Rudder(Node):
    """Driver interface for rudder
    Listens to:
        - /cmd_rudder (String): Sets the rudder to the specified angle (within config limits)
        - /cmd_rudder_offset (String): Shifts the default angle
    """

    MIN_ANGLE = int(c.config["RUDDER"]["min_angle"])
    MAX_ANGLE = int(c.config["RUDDER"]["max_angle"])

    def __init__(self):
        super().__init__("rudder")
        self.logging = self.get_logger()
        self.logging.info("Initializing rudder")

        self.odrive = Odrive(preset="rudder")
        self.sub = self.create_subscription(Float32, "cmd_rudder", self.rudder_callback, 10)
        self.offset_sub = self.create_subscription(String, "offset_rudder", self.offset_callback, 10)

    def rudder_callback(self, msg):
        angle = float(msg.data)

        if angle < self.MIN_ANGLE:
            angle = self.MIN_ANGLE
        elif angle > self.MAX_ANGLE:
            angle = self.MAX_ANGLE

        self.logging.info(f"Moving rudder to {angle} degrees")
        rotations = boatMath.remap(angle, self.MIN_ANGLE, self.MAX_ANGLE, -self.odrive.max_rotations / 2, self.odrive.max_rotations / 2)
        self.odrive.pos = rotations

    def offset_callback(self, msg):
        val = int(msg.data)

        if val < -50:
            val = -50
        elif val > 50:
            val = 50

        self.logging.info(f"Offsetting rudder by {2 * val}%")
        rotations = boatMath.remap(val, -50, 50, -self.odrive.max_rotations / 2, self.odrive.max_rotations / 2)
        self.odrive.pos = rotations


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/rudder"
    rclpy.init(args=args)

    rudder = Rudder()
    rclpy.spin(rudder)


if __name__ == "__main__":
    main()
