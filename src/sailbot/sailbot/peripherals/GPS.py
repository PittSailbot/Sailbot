"""
interfaces with USB GPS sensor
"""
import logging

# https://learn.adafruit.com/adafruit-ultimate-gps/circuitpython-parsing
import time

import gpsd
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from src.sailbot.sailbot.utils.boatMath import convertDegMinToDecDeg
from src.sailbot.sailbot.utils.utils import singleton


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
            logging.debug(f"GPS: {self.latitude} {self.longitude}")
        else:
            logging.warning(f"No GPS fix")

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

    def updategps(self, print_info=False):
        # TODO: Delete
        # must be called before reading latitude and longitude, just pulls data from the sensor
        # optionally prints data read from sensor
        self.gps.update()
        if self.gps.has_fix:
            self.latitude = self.gps.latitude
            self.longitude = self.gps.longitude
            self.track_angle_deg = self.gps.track_angle_deg
        if print_info:
            # self.logging.info(self.latitude,self.longitude, self.gps.latitude,self.gps.longitude)
            if not self.gps.has_fix:
                self.logging.info("Waiting for fix")
                return

            self.logging.info(
                "Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}".format(
                    self.gps.timestamp_utc.tm_mon,  # Grab parts of the time from the
                    self.gps.timestamp_utc.tm_mday,  # struct_time object that holds
                    self.gps.timestamp_utc.tm_year,  # the fix time.  Note you might
                    self.gps.timestamp_utc.tm_hour,  # not get all data like year, day,
                    self.gps.timestamp_utc.tm_min,  # month!
                    self.gps.timestamp_utc.tm_sec,
                )
            )

            self.logging.info("Latitude: {0:.8f} degrees".format(self.gps.latitude))
            self.logging.info("Longitude: {0:.8f} degrees".format(self.gps.longitude))
            self.logging.info("Lat in decDeg:", convertDegMinToDecDeg(self.gps.latitude))
            self.logging.info("long in decDeg:", convertDegMinToDecDeg(self.gps.longitude))
            self.logging.info("Fix quality: {}".format(self.gps.fix_quality))
            self.latitude = self.gps.latitude
            self.longitude = self.gps.longitude
            # Some attributes beyond latitude, longitude and timestamp are optional
            # and might not be present.  Check if they're None before trying to use!
            """if gps.satellites is not None:
                print('# satellites: {}'.format(self.gps.satellites))
            if gps.altitude_m is not None:
                print('Altitude: {} meters'.format(self.gps.altitude_m))
            if gps.speed_knots is not None:
                print('Speed: {} knots'.format(self.gps.speed_knots))
            if gps.track_angle_deg is not None:
                print('Track angle: {} degrees'.format(self.gps.track_angle_deg))
            if gps.horizontal_dilution is not None:
                print('Horizontal dilution: {}'.format(self.gps.horizontal_dilution))
            if gps.height_geoid is not None:
                print('Height geo ID: {} meters'.format(self.gps.height_geoid))
            """


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
        gps.updategps(print_info=True)
        sleep(1)
