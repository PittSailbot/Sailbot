"""
Drivers and interface for camera servos
"""
# Code adapted from https://github.com/ArduCAM/PCA9685
import adafruit_servokit
from rclpy.node import Node

from src.sailbot.sailbot import constants as c
from src.sailbot.sailbot.utils.utils import singleton


@singleton
class CameraServos:
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

    # Servo connection ports, if inputs are reversed then switch
    # If servos don't move try setting ports to 2 and 3
    PITCH_PORT = int(c.config["CAMERASERVOS"]["pitch_port"])
    YAW_PORT = int(c.config["CAMERASERVOS"]["yaw_port"])

    # IS_FLIPPED_PITCH = bool(c.config["CAMERA"]["reverse_pitch"])

    def __init__(self):
        self._kit = adafruit_servokit.ServoKit(channels=16)
        self._pitch = self.DEFAULT_ANGLE
        self._yaw = self.DEFAULT_ANGLE

        self._node = Node("cameraServos")
        self.logging = self._node.get_logger()

        self.logging.info("Initializing camera servos")
        self.reset()

    def __del__(self):
        self.reset()

    def reset(self):
        """Return camera servos to center"""
        self.pitch = self.DEFAULT_ANGLE
        self.yaw = self.DEFAULT_ANGLE

    @property
    def pitch(self):
        return self._kit.servo[self.PITCH_PORT].angle

    @pitch.setter
    def pitch(self, angle):
        if angle < self.MIN_ANGLE:
            angle = self.MIN_ANGLE
        elif angle > self.MAX_ANGLE:
            angle = self.MAX_ANGLE

        # if self.IS_FLIPPED_PITCH:
        # angle = 180 - angle
        self.logging.debug(f"Moving camera pitch to {angle}")
        self._kit.servo[self.PITCH_PORT].angle = angle

    @property
    def yaw(self):
        return self._kit.servo[self.YAW_PORT].angle

    @yaw.setter
    def yaw(self, angle):
        if angle < self.MIN_ANGLE:
            angle = self.MIN_ANGLE
        elif angle > self.MAX_ANGLE:
            angle = self.MAX_ANGLE
        self.logging.debug(f"Moving camera yaw to {angle}")
        self._kit.servo[self.YAW_PORT].angle = angle


if __name__ == "__main__":
    servos = CameraServos()

    while True:
        servos.pitch = int(input("Enter pitch: "))
        servos.yaw = int(input("Enter yaw: "))

        print(servos.pitch)
        print(servos.yaw)
