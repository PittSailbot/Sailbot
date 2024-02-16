"""
Handles interfacing with the I2C compass and accelerometer sensor
"""
import math
from time import sleep
import os

import adafruit_lis2mdl
import adafruit_lsm303_accel
import adafruit_lsm303dlh_mag
import board
import busio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


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
        super().__init__("compass")
        self.logging = self.get_logger()

        # Setup I2C connections
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
            self.accel = adafruit_lsm303_accel.LSM303_Accel(i2c)
        except ValueError as e:
            raise ValueError(f"Failed to initialize compass! Is it plugged in?\n{e}")

        self.pub = self.create_publisher(String, "compass", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.logging.info("Initialized compass")

        self.compassAngle = 0
        self.errcnt = 0
        self.averagedAngle = 0
        self.angle_to_north = 0

    def timer_callback(self):
        # TODO: Calculate heading, pitch & roll from mag
        try:
            mag_x = self.mag.magnetic[0]
            mag_y = self.mag.magnetic[1]
            mag_z = self.mag.magnetic[2]
            accel_x = self.accel.acceleration[0]
            accel_y = self.accel.acceleration[1]
            accel_z = self.accel.acceleration[2]

            roll = math.atan2(accel_y, accel_z)
            pitch = math.atan2(-accel_x, math.sqrt(accel_y * accel_y + accel_z * accel_z))

            heading = math.atan2(mag_y, mag_x)

            heading_degrees = math.degrees(heading)

            angle_to_north = heading_degrees
        except Exception as e:
            raise e

        msg = String(data=str(self.angle_to_north))
        log_msg = ("Angle to North: %0.3f\n" % self.angle_to_north + "Acceleration (m/s^2)): X=%0.3f Y=%0.3f Z=%0.3f\n" % self.accel.acceleration) + "Magnetometer (micro-Teslas)): X=%0.3f Y=%0.3f Z=%0.3f" % self.mag.magnetic
        self.logging.info(f"Publishing: {log_msg}")
        self.pub.publish(msg)


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/compass"
    rclpy.init(args=args)

    comp = Compass()

    rclpy.spin(comp)


if __name__ == "__main__":
    main()
