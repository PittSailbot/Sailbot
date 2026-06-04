# WaveShare RTK GPS
import json
import os

import serial
import socket
import base64
import time
import threading
import glob

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String

gps_data = {
    "fix": False,
    "satellites": 0,
    "hdop": 0.0,
    "latitude": 0.0,
    "longitude": 0.0,
    "speed": 0.0,
    "heading": 0.0,
    "fix_type": 0,
}


class RTK(Node):
    def __init__(self):
        super().__init__("rtk")
        self.logging = self.get_logger()

        # TODO: Initialize any variables / setup needed for the RTK sensor
        # ex. look for the Serial Port (USB) that the sensor is connected on
        # find_gps_port() from the demo code

        self.gps_pub = self.create_publisher(String, "/GPS", 1)
        self.speed_pub = self.create_publisher(Int32, "/speed", 1)

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.last_fix = time.time()

    def timer_callback(self):
        # TODO: Write code to read from the sensor GPS and RTK
        # Should be in ntrip_thread() and gps_thread() from the demo code

        # TODO: Publish the data read from the sensor
        # this syntax is for the old sensor, so you'll likely need to change some of this
        if self.gps.has_fix:
            msg = String()
            msg.data = json.dumps({"lat": self.gps.latitude, "lon": self.gps.longitude, "track_angle": self.gps.track_angle_deg, "velocity": self.gps.speed_knots})
            self.logging.info(str(msg.data))
            self.gps_pub.publish(msg)
            self.logging.debug(f'GPS Publishing: "{msg.data}"')
            self.last_fix = time.time()
        else:
            self.logging.warning(f"Waiting for fix ({round(time.time() - self.last_fix)}s)", throttle_duration_sec=5)


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/rtk"
    rclpy.init(args=args)
    rtk = RTK()
    rclpy.spin(rtk)
