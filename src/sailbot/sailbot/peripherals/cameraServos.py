"""
Drivers and interface for camera servos
"""
# Code adapted from https://github.com/ArduCAM/PCA9685
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import smbus2 as smbus
import os

from sailbot import constants as c
from sailbot.utils.utils import CameraServoState

TEENSY_ADDRESS = int(c.config["MAIN"]["teensy_i2c_address"], 0)
CAM_MOVE_ABS_CMD = int(c.config["CAMERASERVOS"]["CAM_MOVE_CMD"])
CAM_V_MOVE_ABS_CMD = int(c.config["CAMERASERVOS"]["CAM_V_MOVE_ABS_CMD"])
CAM_H_MOVE_ABS_CMD = int(c.config["CAMERASERVOS"]["CAM_H_MOVE_ABS_CMD"])


class CameraServos(Node):
    """
    Drivers and interface for camera servos

    Attributes:
        - pitch: camera pitch
        - yaw: camera yaw

    Functions:
        - reset(): returns camera servos to center
    """

    # Yaw and Pitch assumed to have same range limits
    MIN_ANGLE = int(c.config["CAMERASERVOS"]["min_angle"])
    MAX_ANGLE = int(c.config["CAMERASERVOS"]["max_angle"])
    DEFAULT_ANGLE = int(c.config["CAMERASERVOS"]["default_angle"])

    def __init__(self):
        super().__init__("camera_servos")
        self.logging = self.get_logger()

        self.logging.debug("Initializing camera servos")

        self.bus = smbus.SMBus(1)
        self.reset()

        self.servo_sub = self.create_subscription(String, "cam_servo_control", self.ROS_servo_control_callback, 10)

    def __del__(self):
        self.reset()

    def reset(self):
        """Return camera servos to center"""
        pitch = self.DEFAULT_ANGLE
        yaw = self.DEFAULT_ANGLE
        self.setPosition(CameraServoState(yaw, pitch))

    def setPosition(self, position: CameraServoState):
        if position.horizonal_pos == "" and position.vertical_pos != "":
            data_to_send = [int(CAM_V_MOVE_ABS_CMD), int(position.vertical_pos)]
        elif position.vertical_pos == "" and position.horizonal_pos != "":
            data_to_send = [int(CAM_H_MOVE_ABS_CMD), int(position.horizonal_pos)]
        elif position.horizonal_pos != "" and position.vertical_pos != "":
            data_to_send = [int(CAM_MOVE_ABS_CMD), int(position.horizonal_pos), int(position.vertical_pos)]
        self.send_data_to_teensy(data_to_send)

    def send_data_to_teensy(self, data: list):
        if len(data) > 32:
            print("Teensy is only configured to store 32 bytes of data per chunk")
        try:
            # Send a block of data to Teensy
            print(data)
            self.bus.write_i2c_block_data(TEENSY_ADDRESS, 0, data)
        except IOError as e:
            print("Error sending data to Teensy: ", e)

    def ROS_servo_control_callback(self, message):
        pos = CameraServoState.fromRosMessage(message)
        self.setPosition(pos)


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/cameraServos"
    rclpy.init(args=args)

    servos = CameraServos()
    rclpy.spin(servos)


if __name__ == "__main__":
    main()
