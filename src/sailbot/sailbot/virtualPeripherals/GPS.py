"""
interfaces with USB GPS sensor
"""

from sailbot.boatMath import (
    degreesToRadians,
    getCoordinateADistanceAlongAngle,
    distanceInMBetweenEarthCoordinates,
    computeNewCoordinate,
    angleBetweenCoordinates,
    convertDegMinToDecDeg,
    convertWindAngle,
)
from sailbot.utils import dummyObject

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class gps(Node):
    """
    Attributes:
        latitude (float)
        longitude (float)
    """

    def __init__(self):
        self.gps = dummyObject()
        self.gps.latitude = 0
        self.gps.longitude = 0
        self.gps.track_angle_deg = 0

        super().__init__("GPS")
        self.logging = self.get_logger()
        self.pub = self.create_publisher(String, "GPS", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f"{self.gps.latitude},{self.gps.longitude},{self.gps.track_angle_deg}"
        self.pub.publish(msg)
        self.logger.info('Publishing: "%s"' % msg.data)

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
    GPS = gps()
    rclpy.spin(GPS)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    GPS.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    GPS = gps()
    while True:
        GPS.updategps()
        sleep(1)
