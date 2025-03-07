"""
Handles interfacing with the I2C compass and accelerometer sensor
"""

import json
import math
import os
from time import sleep

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from sailbot import constants as c
from sailbot.utils.utils import ImuData


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
        self.rudder_angle = 0
        self.velocity = 0.0
        self.rudder_sub = self.create_subscription(Float32, "cmd_rudder", self.rudder_callback, 10)
        self.pub = self.create_publisher(String, "imu", 10)
        self.gps_subscription = self.create_subscription(String, "GPS", self.ROS_GPSCallback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.compassAngle = 90

    def ROS_GPSCallback(self, string):
        string = string.data
        gpsJson = json.loads(string)
        self.velocity = gpsJson["velocity"]

    def rudder_callback(self, msg):
        self.rudder_angle = float(msg.data)

    def timer_callback(self):
        if abs(self.rudder_angle) > float(c.config["RUDDER"]["acceptable_error"]):
            self.compassAngle -= (self.rudder_angle / 10) * self.velocity
            self.compassAngle %= 360

        msg = ImuData(self.compassAngle, 0, 0).toRosMessage()
        self.pub.publish(msg)
        self.logging.debug('Publishing: "%s"' % msg.data)

    @property
    def angle(self):
        # returns smoothed angle measurement
        return self.compassAngle


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/compass"
    rclpy.init(args=args)
    comp = Compass()
    rclpy.spin(comp)
