"""
Handles interfacing with the I2C compass and accelerometer sensor
"""
import math
from time import sleep

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
        # Setup I2C connections
        i2c = busio.I2C(board.SCL, board.SDA)
        self.mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
        self.accel = adafruit_lsm303_accel.LSM303_Accel(i2c)
        self.compassAngle = 0
        self.errcnt = 0

        self.averagedAngle = 0

        super().__init__("Compass")
        self.pub = self.create_publisher(String, "compass", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f"{self.angle}"
        self.pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    @property
    def vector(self):
        return sensor.magnetic  # (mag_x, mag_y, mag_z)

    @property
    def angle_X(self):
        # return X component of compass, occasionally the compass fails to read, if this happens 10 times in a row raise error
        try:
            self.compassAngle1 = self.mag.magnetic[0]
            self.errcnt = 0
        except:
            self.errcnt += 1
            if self.errcnt > 10:
                raise Exception("DISCONNECTED COMPASS")

        return self.compassAngle1

    @property
    def angle_Y(self):
        # return Y component of compass, occasionally the compass fails to read, if this happens 10 times in a row raise error
        try:
            self.compassAngle2 = self.mag.magnetic[1]  # issues
            self.errcnt = 0
        except:
            self.errcnt += 1
            if self.errcnt > 10:
                raise Exception("DISCONNECTED COMPASS")

        return self.compassAngle2

    @property
    def angle_Z(self):
        # return Z component of compass, occasionally the compass fails to read, if this happens 10 times in a row raise error
        try:
            self.compassAngle3 = self.mag.magnetic[2]  # issues
            self.errcnt = 0
        except:
            self.errcnt += 1
            if self.errcnt > 10:
                raise Exception("DISCONNECTED COMPASS")

        return self.compassAngle3

    @property
    def angle_to_north(self):
        return math.atan2(-self.angle_Y, -self.angle_X) * 180 / math.pi

    @property
    def angle(self):
        # returns smoothed angle measurement
        alpha = 0.9
        self.averagedAngle = self.averagedAngle * alpha + self.angle_to_north * (1 - alpha)
        return self.averagedAngle

    def printAccel(self):
        self.logging.info("Acceleration (m/s^2)): X=%0.3f Y=%0.3f Z=%0.3f" % self.accel.acceleration)

    def printMag(self):
        self.logging.info("Magnetometer (micro-Teslas)): X=%0.3f Y=%0.3f Z=%0.3f" % self.mag.magnetic)
        self.logging.info(f"Angle {self.angle}")
        # self.logging.info(F"angle to north: {self.angleToNorth}")


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/compass"
    rclpy.init(args=args)
    comp = Compass()
    rclpy.spin(comp)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    comp.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    comp = Compass()
    while True:
        comp.printMag()
        sleep(0.2)
