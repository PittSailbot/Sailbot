"""
interfaces with USB GPS sensor
"""

import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

from sailbot.utils.utils import DummyObject, ControlState, ImuData
from sailbot.utils.boatMath import is_within_angle
import random
import math
import json


NO_GO_MIN = 40
NO_GO_MAX = 360 - NO_GO_MIN

MAX_VEL = 1.5 #m/s
MAX_ACCEL = 0.01 #m/s^2
MAX_DECCEL = 0.02

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
        self.velocity = 0.0
        self.compass_yaw = 0.0
        self.relative_wind = 0.0
        self.rudder_angle = 0.0
        self.sail_angle = 0.0
        self.gps.track_angle_deg = -1

        super().__init__("GPS")
        self.logging = self.get_logger()
        self.pub = self.create_publisher(String, "GPS", 10)
        self.control_state_pub = self.create_publisher(String, "control_state", 10)

        self.compass_subscription = self.create_subscription(
            String, "imu", self.ROS_compassCallback, 10
        )
        self.windvane_subscription = self.create_subscription(
            String, "wind_angle", self.ROS_windvaneCallback, 10
        )
        
        self.sail_sub = self.create_subscription(Float32, "cmd_sail", self.sail_callback, 10)
        
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.logging.info("GPS Started")

    def sail_callback(self, msg):
        self.sail_angle = float(msg.data)

    def ROS_windvaneCallback(self, string):
        angle = float(string.data)
        self.relative_wind = angle

    def ROS_compassCallback(self, string):
        data = ImuData.fromRosMessage(string)
        self.compass_yaw = data.yaw

    def timer_callback(self):
        self.control_state_pub.publish(ControlState(ControlState.AUTO, ControlState.AUTO).toRosMessage())
        
        optAngle = max(min(self.relative_wind / 2, 90), 3)
        self.sail_angle = optAngle
        if ( is_within_angle(self.relative_wind, (NO_GO_MIN), (NO_GO_MAX))):
            self.velocity = min(MAX_VEL, self.velocity + MAX_ACCEL) * max(
                (1 - abs(self.sail_angle - optAngle) / 30), 0
            )
        else:
            self.velocity = max(self.velocity - MAX_DECCEL, 0) * max(
                (1 - abs(self.sail_angle - optAngle) / 30), 0
            )

        dx, dy = self.calculate_position_change(self.velocity, degreesToRadians(self.compass_yaw))
        self.gps.latitude, self.gps.longitude = self.computeNewCoordinate(self.gps.latitude, self.gps.longitude, dx, dy)
        gpsNoise = 0
        noisyLat, noisyLon = self.computeNewCoordinate(self.gps.latitude, self.gps.longitude, self.getNoise(gpsNoise), self.getNoise(gpsNoise))

        msg = String()
        msg.data = (
            json.dumps({'lat': noisyLat,
                        'lon': noisyLon,
                        'track_angle': self.gps.track_angle_deg,
                        'velocity': self.velocity})
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

    def getNoise(self, max_noise):
        return random.random() * max_noise - max_noise / 2

    def calculate_position_change(self, velocity, yaw):
        """
        Calculate change in x and y position given yaw (angle) and velocity.

        Parameters:
        yaw (float): Yaw angle in radians.
        velocity (float): Velocity in m/s.

        Returns:
        tuple: Change in x and y position (delta_x, delta_y).
        """
        delta_x = velocity * math.cos(yaw)
        delta_y = velocity * math.sin(yaw)
        return delta_x, delta_y

    def computeNewCoordinate(self, lat, lon, d_lat, d_lon):
        """
        finds the gps coordinate that is x meters from given coordinate
        d_lat, d_lon in meters
        """
        earthRadiusKm = 6371

        d_lat /= 1000
        d_lon /= 1000

        new_lat = lat + (d_lat / earthRadiusKm) * (180 / math.pi)
        new_lon = lon + (d_lon / earthRadiusKm) * (180 / math.pi) / math.cos(
            lat * math.pi / 180
        )

        return (new_lat, new_lon)
    
def degreesToRadians(degrees):
    return degrees * math.pi / 180


def radiansToDegrees(rads):
    return rads * 180 / math.pi


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
