import os
import subprocess
import sys
from time import sleep

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sailbot.utils.utils import EventLaunchDescription, UsbResetCmd


class ActionManager(Node):
    def __init__(self):
        super().__init__("ActionManager")
        self.logging = self.get_logger()

        self.event_shutdown_pub = self.create_publisher(String, "event_shutdown", 10)

        self.event_change_subscription = self.create_subscription(String, "setEvent", self.ROS_eventChange_callback, 10)

        self.usbReset_subscription = self.create_subscription(String, "usbReset", self.ROS_USB_Reset, 1)

        self.logging.info("starting Action Manager")

    def ROS_eventChange_callback(self, msg):
        self.event_shutdown_pub.publish(String(data=""))
        desc = EventLaunchDescription.fromRosMessage(msg)
        self.logging.info(str(desc))
        if desc.paramsFile:
            subprocess.Popen(["ros2", "launch", "sailbot", "event_launch.launch.py", "--", f"executable:={desc.eventExecutable}", f"paramsFile:={desc.paramsFile}"])
        else:
            subprocess.Popen(["ros2", "launch", "sailbot", "event_launch.launch.py", "--", f"executable:={desc.eventExecutable}"])

    def ROS_USB_Reset(self, msg):
        """
        Turn off the specified port on the given hub.
        """
        command = ["sudo", "uhubctl", "-l", 2, "-a", "off"]
        result = subprocess.run(command, capture_output=True, text=True)
        command = ["sudo", "uhubctl", "-l", 3, "-a", "off"]
        result = subprocess.run(command, capture_output=True, text=True)
        sleep(1)
        command = ["sudo", "uhubctl", "-l", 2, "-a", "on"]
        result = subprocess.run(command, capture_output=True, text=True)
        command = ["sudo", "uhubctl", "-l", 3, "-a", "on"]
        result = subprocess.run(command, capture_output=True, text=True)

        return result.stdout, result.stderr


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/actionManager"
    rclpy.init(args=args)

    t = ActionManager()
    try:
        rclpy.spin(t)
    finally:
        rclpy.shutdown()
