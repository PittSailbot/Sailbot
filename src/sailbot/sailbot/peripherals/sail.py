"""
Contains drivers for sail and rudder and can autonomously sail to a specific gps position or heading
"""
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sailbot import constants as c
from sailbot.peripherals.Odrive import Odrive
from sailbot.utils import boatMath


class Sail(Node):
    """Driver interface for sail
    Listens to:
        - /cmd_sail (String): Sets the sail to the specified angle (within config limits)
        - /cmd_rudder_offset (String): Shifts the default angle
    """

    MIN_ANGLE = int(c.config["SAIL"]["min_angle"])
    MAX_ANGLE = int(c.config["SAIL"]["max_angle"])

    def __init__(self):
        super().__init__("sail")
        self.logging = self.get_logger()
        self.logging.info("Initializing sail")

        self.odrive = Odrive(preset="sail")
        self.sub = self.create_subscription(String, "cmd_sail", self.sail_callback, 10)
        self.offset_sub = self.create_subscription(String, "offset_sail", self.offset_callback, 10)

    def sail_callback(self, msg):
        angle = int(msg.data)

        if angle < self.MIN_ANGLE:
            angle = self.MIN_ANGLE
        elif angle > self.MAX_ANGLE:
            angle = self.MAX_ANGLE

        self.logging.debug(f"Moving sail to {angle} degrees")
        rotations = boatMath.remap(angle, self.MIN_ANGLE, self.MAX_ANGLE, -self.odrive.max_rotations / 2, self.odrive.max_rotations / 2)
        self.odrive.pos = rotations

    def offset_callback(self, msg):
        val = int(msg.data)

        if val < -50:
            val = -50
        elif val > 50:
            val = 50

        self.logging.info(f"Offsetting sail by {2 * val}%")
        rotations = boatMath.remap(val, -50, 50, -self.odrive.max_rotations / 2, self.odrive.max_rotations / 2)
        self.odrive.pos = rotations


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/sail"
    rclpy.init(args=args)

    sail = Sail()
    rclpy.spin(sail)


if __name__ == "__main__":
    main()
