import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import sys, os


class Test(Node):
    def __init__(self):
        super().__init__("test")
        self.logging = self.get_logger()

        self.pub = self.create_publisher(String, "test", 10)
        self.timer = self.create_timer(1, self.timer_callback)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(20, GPIO.OUT)

        self.pinState = False

        self.logging.info("starting")

    def timer_callback(self):
        self.pinState = not self.pinState
        GPIO.output(20, GPIO.HIGH if self.pinState else GPIO.LOW)
        msg = String(data=str("test"))
        self.logging.info("timer")
        self.pub.publish(msg)


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/rosTest"
    rclpy.init(args=args)

    t = Test()
    rclpy.spin(t)
