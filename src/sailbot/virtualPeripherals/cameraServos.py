"""
Drivers and interface for camera servos
"""
# Code adapted from https://github.com/ArduCAM/PCA9685
from rclpy.node import Node

import sailbot.constants as c
from src.sailbot.utils.utils import singleton


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
        self._pitch = self.DEFAULT_ANGLE
        self._yaw = self.DEFAULT_ANGLE

        self._node = Node("vitualCameraServos")
        self.logging = self._node.get_logger()

        self.logging.debug("Initializing camera servos")
        self.reset()

    def __del__(self):
        self.reset()

    def reset(self):
        """Return camera servos to center"""
        self.pitch = self.DEFAULT_ANGLE
        self.yaw = self.DEFAULT_ANGLE

    # ============ HERE BE DRAGONS ============
    # Python boilerplate for creating implicit setters and getters
    # Instead of writing 'servos.set_pitch(90)' just write 'servos.pitch = 90'
    @property
    def pitch(self):
        return 0

    @pitch.setter
    def pitch(self, angle):
        pass

    @property
    def yaw(self):
        return 0

    @yaw.setter
    def yaw(self, angle):
        pass
