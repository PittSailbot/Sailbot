"""
hadles turning motors using Odrive/Stepper driver 
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sailbot.constants as c
import sailbot.utils as utils
from sailbot.virtualPeripherals.Odrive import Odrive

from threading import Thread
from time import sleep
import os

# define type of motor that is being used
USE_ODRIVE_SAIL = True
USE_STEPPER_SAIL = False

USE_ODRIVE_RUDDER = True
USE_STEPPER_RUDDER = False

if False:  # The two wiring configurations, both defined here for easy switching
    SAIL_DIR_PIN = 17
    SAIL_PUL_PIN = 4
    RUDDER_DIR_PIN = 22
    RUDDER_PUL_PIN = 27
else:
    SAIL_DIR_PIN = 22
    SAIL_PUL_PIN = 27
    RUDDER_DIR_PIN = 17
    RUDDER_PUL_PIN = 4


class obj_sail:
    def __init__(self, parent, auto=False):
        self._node = Node("virtualSail")
        self.logging = self._node.get_logger()
        self.autoAdjust = auto
        self.current = 0
        self.offset = 0

    def set(self, degrees, force=False):
        self.logging.debug(f"setting sail: {degrees}")
        degrees = float(degrees)

        if not force and abs(degrees - self.current) < 3:
            return

        self.current = degrees

    def autoAdjustSail(self):
        while True:
            if self.autoAdjust == True:
                windDir = self.windvane.angle
                if windDir > 180:
                    windDir = 180 - (windDir - 180)
                targetAngle = max(min(windDir / 2, 90), 3)
                self.set(targetAngle)


class obj_rudder:
    # 800 steps = 360 degrees
    # between -45 and 45 degrees
    def __init__(self, parent):
        self._node = parent
        self.logging = self._node.get_logger()
        self.current = 0
        self.offset = 0

    def set(self, degrees, force=False):
        self.logging.debug(f"setting rudder: {degrees}")
        degrees = float(degrees)

        if not force and abs(degrees - self.current) < 3:
            return

        self.current = degrees


class driver(Node):
    def __init__(self, calibrateOdrive=False):
        super().__init__("virtualDriver")
        self.logging = self.get_logger()
        global DRV
        if USE_ODRIVE_SAIL or USE_ODRIVE_RUDDER:
            DRV = Odrive(self, calibrate=calibrateOdrive)
            pass
        self.sail = obj_sail(self)
        self.rudder = obj_rudder(self)

        self.driver_subscription = self.create_subscription(
            String, "driver", self.ROS_Callback, 10
        )

    def ROS_Callback(self, string):
        string = string.data
        # string = (driver:sail/rudder:{targetAngle})
        self.logging.debug(f"driver callback {string}")
        resolved = False
        args = string.replace("(", "").replace(")", "").split(":")
        if args[0] == "driver":
            if args[1] == "sail":
                self.sail.set(float(args[2]))
                resolved = True
            elif args[1] == "rudder":
                self.rudder.set(float(args[2]))
                resolved = True

        elif args[0] == "driverOffset":
            if args[1] == "sail":
                newVal = float(args[2])
                if abs(self.sail.offset - newVal) > 0.15:
                    self.sail.offset = newVal
                    self.sail.set(self.sail.current, force=True)
                    self.logging.debug(f"Sail offset = {self.sail.offset}")
                resolved = True
            elif args[1] == "rudder":
                self.rudder.offset = float(args[2])
                self.rudder.set(self.rudder.current, force=True)
                resolved = True

        if not resolved:
            self.logging.warning(
                f"driver failed to resolve command: {string}, parsed to {args}"
            )


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + f"/drivers"
    rclpy.init(args=args)
    drv = driver()
    try:
        rclpy.spin(drv)
    except Exception as e:
        drv.logging.debug(f"exception rased in driver {e}")
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drv.logging.debug("Destroying driver node")
    drv.destroy_node()
    rclpy.shutdown()
