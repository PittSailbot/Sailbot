"""
interfaces with USB GPS sensor
"""

import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sailbot.utils.utils import DummyObject
import random

from sailbot.utils.utils import singleton


@singleton
class GPS(Node):
    """
    Attributes:
        latitude (float)
        longitude (float)
    """

    def __init__(self):
        self.gps = DummyObject()
        self.gps.latitude = 42.84963
        self.gps.longitude = -70.986314
        self.gps.track_angle_deg = -1

        super().__init__("GPS")
        self.logging = self.get_logger()
        self.pub = self.create_publisher(String, "GPS", 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.logging.info("GPS Started")

    def timer_callback(self):

        self.gps.longitude += 0.00001
        if self.gps.longitude > -70.972903:
            self.gps.longitude = -70.986314
            self.gps.latitude += 0.0005
        randomOffset = 0.00005

        noisyLat =  self.gps.latitude + random.random() * randomOffset - randomOffset / 2
        noisyLon = self.gps.longitude + random.random() * randomOffset - randomOffset / 2


        msg = String()
        msg.data = (
            f"{noisyLat},{noisyLon},{self.gps.track_angle_deg}"
        )
        self.pub.publish(msg)
        self.logging.debug(F'GPS Publishing: "{msg.data}"')

    def __getattribute__(self, name):
        """
        if an attempt is made to access an attribute that does not exist, it will then attempt to get the attribute from gps object
        this means that rather than using gps_object.gps.longitude, it is possible to use gps_object.longitude
        """
        try:
            return super().__getattribute__(name)
        except:
            return self.gps.__getattribute__(name)


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/gps"
    rclpy.init(args=args)
    gps = GPS()
    rclpy.spin(gps)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    from time import sleep
    gps = GPS()
    while True:
        gps.updategps()
        sleep(1)
