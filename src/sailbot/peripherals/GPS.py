"""
interfaces with USB GPS sensor
"""
# https://learn.adafruit.com/adafruit-ultimate-gps/circuitpython-parsing
from time import sleep
import os

import gpsd
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from src.sailbot.utils.utils import singleton


@singleton
class GPS(Node):
    """
    Attributes:
        latitude (float): the current latitude of the boat
        longitude (float): the current longitude of the boat
    """

    def __init__(self):
        self.latitude = None
        self.longitude = None
        self._node = Node("GPS")
        self.logging = self._node.get_logger()

        # Can also use adafruit_gps module (but didn't work at competition?)
        # Prev GPS version w/ adafruit: https://github.com/SailBotPitt/SailBot/blob/a56a18b06cbca78aace1990a4a3ce4dfd8c7a847/GPS.py
        # GPSD doesn't have gps track angle degree
        gpsd.connect()
        packet = gpsd.get_current()
        if packet.mode >= 2:
            self.latitude = packet.position()[0]
            self.longitude = packet.position()[1]
            self.logging.debug(f"GPS: {self.latitude} {self.longitude}")
        else:
            self.logging.warning(f"No GPS fix")

        super().__init__("GPS")
        self.pub = self.create_publisher(String, "GPS", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        packet = gpsd.get_current().position()
        self.latitude = packet[0]
        self.longitude = packet[1]
        msg.data = f"{self.latitude},{self.longitude}"
        self.pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

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
    gps = gps()
    rclpy.spin(gps)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    rclpy.init()
    gps = GPS()
    rclpy.spin(gps)
    while True:
        # GPS.updategps() Deprecated function
        sleep(1)
