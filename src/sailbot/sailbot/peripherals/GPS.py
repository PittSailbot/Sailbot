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
        if gpsd.get_current().mode < 2:
            self.logging.warning(f"No GPS fix")

        super().__init__("GPS")
        self.pub = self.create_publisher(String, "GPS", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        packet = gpsd.get_current()

        if packet.mode >= 2:
            self.latitude = packet.position()[0]
            self.longitude = packet.position()[1]

            msg.data = f"{self.latitude},{self.longitude}"
            self.pub.publish(msg)
            self.logging.info('Publishing: "%s"' % msg.data)
        else:
            self.logging.warning(f"No GPS fix")

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


if __name__ == "__main__":
    main()
