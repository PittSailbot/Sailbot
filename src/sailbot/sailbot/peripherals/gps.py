# sudo pip3 install adafruit-circuitpython-gps

# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import json
import os

# Simple GPS module demonstration.
# Will wait for a fix and print a message every second with the current location
# and other details.
import time
from time import sleep

import adafruit_gps
import board
import busio
import rclpy
import serial
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from serial.tools import list_ports
from std_msgs.msg import Float32, Int32, String

from sailbot import constants as c
from sailbot.telemetry.protobuf import controlsData_pb2, teensy_pb2
from sailbot.utils import boatMath

# from geographic_msgs.msg import GeoPose, GeoPoint


# https://www.geeksforgeeks.org/how-to-install-protocol-buffers-on-windows/
# Compile .proto with `protoc teensy.proto --python_out=./`
# from sailbot.utils.utils import Waypoint, ControlState, ImuData


class GPS(Node):
    def __init__(self):
        super().__init__("gps")
        self.logging = self.get_logger()

        self.uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)

        self.gps = adafruit_gps.GPS(self.uart, debug=False)  # Use UART/pyserial

        self.gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")

        self.gps.send_command(b"PMTK220,1000")

        self.pub = self.create_publisher(String, "/boat/GPS", 1)

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.gps.update()

        if self.gps.has_fix:
            msg = String()
            msg.data = json.dumps({"lat": self.gps.latitude, "lon": self.gps.longitude, "track_angle": self.gps.track_angle_deg, "velocity": self.gps.speed_knots})
            self.logging.info(str(msg.data))
            self.pub.publish(msg)
            self.logging.debug(f'GPS Publishing: "{msg.data}"')

    # # Main loop runs forever printing the location, etc. every second.
    # last_print = time.monotonic()
    # while True:
    #     # Make sure to call gps.update() every loop iteration and at least twice
    #     # as fast as data comes from the GPS unit (usually every second).
    #     # This returns a bool that's true if it parsed new data (you can ignore it
    #     # though if you don't care and instead look at the has_fix property).
    #     gps.update()
    #     # Every second print out current location details if there's a fix.
    #     current = time.monotonic()
    #     if current - last_print >= 1.0:
    #         last_print = current
    #         if not gps.has_fix:
    #             # Try again if we don't have a fix yet.
    #             print("Waiting for fix...")
    #             continue
    #         # We have a fix! (gps.has_fix is true)
    #         # Print out details about the fix like location, date, etc.
    #         print("=" * 40)  # Print a separator line.
    #         print(
    #             "Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}".format(
    #                 gps.timestamp_utc.tm_mon,  # Grab parts of the time from the
    #                 gps.timestamp_utc.tm_mday,  # struct_time object that holds
    #                 gps.timestamp_utc.tm_year,  # the fix time.  Note you might
    #                 gps.timestamp_utc.tm_hour,  # not get all data like year, day,
    #                 gps.timestamp_utc.tm_min,  # month!
    #                 gps.timestamp_utc.tm_sec,
    #             )
    #         )
    #         print("Latitude: {0:.6f} degrees".format(gps.latitude))
    #         print("Longitude: {0:.6f} degrees".format(gps.longitude))
    #         print(
    #             "Precise Latitude: {} degs, {:2.4f} mins".format(
    #                 gps.latitude_degrees, gps.latitude_minutes
    #             )
    #         )
    #         print(
    #             "Precise Longitude: {} degs, {:2.4f} mins".format(
    #                 gps.longitude_degrees, gps.longitude_minutes
    #             )
    #         )
    #         print("Fix quality: {}".format(gps.fix_quality))
    #         # Some attributes beyond latitude, longitude and timestamp are optional
    #         # and might not be present.  Check if they're None before trying to use!
    #         if gps.satellites is not None:
    #             print("# satellites: {}".format(gps.satellites))
    #         if gps.altitude_m is not None:
    #             print("Altitude: {} meters".format(gps.altitude_m))
    #         if gps.speed_knots is not None:
    #             print("Speed: {} knots".format(gps.speed_knots))
    #         if gps.track_angle_deg is not None:
    #             print("Track angle: {} degrees".format(gps.track_angle_deg))
    #         if gps.horizontal_dilution is not None:
    #             print("Horizontal dilution: {}".format(gps.horizontal_dilution))
    #         if gps.height_geoid is not None:
    #             print("Height geoid: {} meters".format(gps.height_geoid))'


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/gps"
    rclpy.init(args=args)
    gps = GPS()
    rclpy.spin(gps)
