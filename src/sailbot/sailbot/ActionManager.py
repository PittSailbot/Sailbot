import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys, os, subprocess

from sailbot.utils.utils import EventLaunchDescription

class ActionManager(Node):
    def __init__(self):
        super().__init__("ActionManager")
        self.logging = self.get_logger()

        self.event_shutdown_pub = self.create_publisher(String, "event_shutdown", 10)

        self.event_change_subscription = self.create_subscription(
            String, "setEvent", self.ROS_eventChange_callback, 10
        )

        self.logging.info("starting Action Manager")
        
    def ROS_eventChange_callback(self, msg):
        self.event_shutdown_pub.publish(String(data=""))
        desc = EventLaunchDescription.fromRosMessage(msg)
        self.logging.info(str(desc))
        if desc.paramsFile:
            subprocess.Popen(['ros2', 'launch', 'sailbot', 'event_launch.launch.py', '--', F'executable:={desc.eventExecutable}', F'paramsFile:={desc.paramsFile}'])
        else:
            subprocess.Popen(['ros2', 'launch', 'sailbot', 'event_launch.launch.py', '--', F'executable:={desc.eventExecutable}'])

def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/actionManager"
    rclpy.init(args=args)

    t = ActionManager()
    try:
        rclpy.spin(t)
    finally:
        rclpy.shutdown()