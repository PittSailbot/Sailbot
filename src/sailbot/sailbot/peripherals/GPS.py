"""
Interfaces with USB GPS sensor
"""
# https://learn.adafruit.com/adafruit-ultimate-gps/circuitpython-parsing
import os

import gpsd
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GPS(Node):
    """
    Publishes to:
        - /gps: (String) '40.321,20.321'
    """

    def __init__(self):
        super().__init__("gps")
        self.logging = self.get_logger()

        # Can also use adafruit_gps module (but didn't work at competition?)
        # Prev GPS version w/ adafruit: https://github.com/SailBotPitt/SailBot/blob/a56a18b06cbca78aace1990a4a3ce4dfd8c7a847/GPS.py
        # GPSD doesn't have gps track angle degree
        gpsd.connect()
        if gpsd.get_current().mode < 2:
            self.logging.warning(f"No GPS fix")

        self.pub = self.create_publisher(String, "gps", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        packet = gpsd.get_current()

        if packet.mode >= 2:
            latitude = packet.position()[0]
            longitude = packet.position()[1]

            msg = f"{latitude},{longitude}"
            self.logging.info(f"Publishing: '{msg}'")
            self.pub.publish(String(msg))

        else:
            self.logging.warning(f"No GPS fix")


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/gps"
    rclpy.init(args=args)

    gps = GPS()
    rclpy.spin(gps)


if __name__ == "__main__":
    main()
