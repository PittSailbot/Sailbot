"""
Interface for reading wind angle
"""
from threading import Lock, Thread
from time import sleep
import os 
import json
    
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sailbot import constants as c


class WindVane(Node):
    """Measures the angle of the wind

    Attributes:
        angle (float): the angle pointing into the wind
        no_go_zone (NoGoZone): the left and right limits of the no go zone
    """

    def __init__(self):
        super().__init__("WindVane")
        self.logging = self.get_logger()

        self.wind_source_angle = 0.0
        self.compass_yaw = 0.0

        self.pub = self.create_publisher(String, "windvane", 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.compass_subscription = self.create_subscription(
            String, "/boat/compass", self.ROS_compassCallback, 10
        )

        self._angle = 0

    def ROS_compassCallback(self, string):
        string = string.data
        if string == "None,None":
            self.compass_yaw = 0.0
            return

        angle = string.replace("(", "").replace(")", "")
        self.compass_yaw = float(angle)

    def timer_callback(self):

        self.angle = (-self.compass_yaw + self.wind_source_angle) % 360

        msg = String()
        msg.data = (
            json.dumps({'angle': self.angle})
        )
        self.pub.publish(msg)
        self.logging.debug(F'Windvane Publishing: "{msg.data}"')

    def map(self, x, min1, max1, min2, max2):
        # converts value x, which ranges from min1-max1, to a corresponding value ranging from min2-max2
        # ex: map(0.3, 0, 1, 0, 100) returns 30
        # ex: map(70, 0, 100, 0, 1) returns .7
        x = min(max(x, min1), max1)
        return min2 + (max2 - min2) * ((x - min1) / (max1 - min1))


class NoGoZone:
    """Simplifies checking whether a compass heading is inside the no go zone
    - Declare a WindVane object and write 'if x in windvane.no_go_zone'

    Attributes:
        left_bound (float): the compass angle of the left-most no go zone bound
        right_bound (float): the compass angle of the right-most no go zone bound
    """

    NO_GO_RANGE = int(c.config["WINDVANE"]["no_go_range"]) / 2

    def __init__(self, wind_direction):
        self.left_bound = (wind_direction - self.NO_GO_RANGE) % 360
        self.right_bound = (wind_direction + self.NO_GO_RANGE) % 360

    def __contains__(self, heading):
        if self.left_bound > self.right_bound:
            heading = heading % 360

            # Check if the heading is within the wrapped bounds
            if heading >= self.left_bound or heading <= self.right_bound:
                return True
        else:
            # Bounds don't wrap around
            if self.left_bound <= heading <= self.right_bound:
                return True

        return False

def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/windvane"
    rclpy.init(args=args)
    windvane = WindVane()
    rclpy.spin(windvane)

if __name__ == "__main__":
    main()

