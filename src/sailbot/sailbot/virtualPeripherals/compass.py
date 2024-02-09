"""
Handles interfacing with the I2C compass and accelerometer sensor
"""
import math
from time import sleep
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Compass(Node):
    """Measure's the boat's heading, acceleration and gyroscopic orientation in all axes.

    Attributes:
        angle_to_north (float):
        accel (float):
        angle_X (float):
        angle_Y (float):
        angle_Z (float):

    """

    def __init__(self):
        super().__init__("Compass")
        self.logging = self.get_logger()

        self.pub = self.create_publisher(String, "compass", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.compassAngle = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"{self.angle}"
        self.pub.publish(msg)
        self.logging.debug('Publishing: "%s"' % msg.data)

    @property
    def angle(self):
        # returns smoothed angle measurement
        self.compassAngle += 10
        return self.compassAngle

def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/compass"
    rclpy.init(args=args)
    comp = Compass()
    rclpy.spin(comp)
