"""Very commonly used utility functions and classes"""

import json
import math
import os
from dataclasses import dataclass
from datetime import datetime

import rclpy
from rclpy.executors import ShutdownException, TimeoutException
from std_msgs.msg import Bool, String

from sailbot import constants as c
from sailbot.utils.boatMath import distance_between, quaternion_to_euler


@dataclass(slots=True)
class Waypoint:
    """
    A GPS marker for buoys, travel destinations, etc.
        - Initialize using Waypoint(latitude, longitude)
    """

    lat: float
    lon: float

    def __str__(self):
        return f"({self.lat}, {self.lon})"

    def __repr__(self):
        return f"Waypoint({self.lat, self.lon})"

    def to_msg(self):
        msg = String()
        msg.data = self.toJson()

        return msg

    def from_msg(jsonString: String):
        if jsonString == "":
            return Waypoint(-1, -1)

        gpsJson = json.loads(jsonString.data)
        if gpsJson["lat"] == None and gpsJson["lon"] == None:
            return None

        return Waypoint(float(gpsJson["lat"]), float(gpsJson["lon"]))

    def add_meters(self, dx, dy):
        """Updates the waypoint gps by adding meters to the latitude and longitude"""
        EARTH_RADIUS = 6371000

        self.lat += (dy / EARTH_RADIUS) * (180 / math.pi)
        self.lon += (dx / EARTH_RADIUS) * (180 / math.pi) / math.cos(self.lat * math.pi / 180)

    def toJson(self):
        return json.dumps({"lat": self.lat, "lon": self.lon})

    def fromJson(json_data):
        if str(json_data).upper() == "NONE":
            return None

        data = json.loads(json_data)
        return Waypoint(data["lat"], data["lon"])


class DummyObject:
    """
    a class that can be used as a placeholder for something else, usually a physical sensor that is not present.
    allows for setting of arbitrary variables that can be read and written to
    """

    def __init__(self, *args, **kwargs):
        pass


class ControlState:
    MANUAL = 1
    AUTO = 2
    EXTERNAL_CONTROL = 3

    def __init__(self, rudder, sail):
        self.rudder = rudder
        self.sail = sail

    @property
    def full_auto(self):
        return self.rudder != self.MANUAL and self.sail != self.MANUAL

    def __repr__(self) -> str:
        if self.sail == self.MANUAL and self.rudder == self.MANUAL:
            return "RC Control"
        elif self.rudder == self.MANUAL:
            return "Auto Sail"
        else:
            return "Autonomous"

    def toRosMessage(self):
        msgData = {"rudder": self.rudder, "sail": self.sail}
        msg = String()
        msg.data = json.dumps(msgData)

        return msg

    @staticmethod
    def fromRosMessage(message):
        data = json.loads(message.data)
        return ControlState(data["rudder"], data["sail"])


class CameraServoState:
    def __init__(self, horizonal_pos: int, vertical_pos: int):
        self.horizonal_pos = horizonal_pos
        self.vertical_pos = vertical_pos

    def toRosMessage(self):
        msgData = {"horizonal_pos": self.horizonal_pos, "vertical_pos": self.vertical_pos}
        msg = String()
        msg.data = json.dumps(msgData)

        return msg

    @staticmethod
    def fromRosMessage(message):
        data = json.loads(message.data)
        return CameraServoState(data["horizonal_pos"], data["vertical_pos"])


class EventLaunchDescription:
    def __init__(self, eventExecutable, paramsFile):
        self.eventExecutable = eventExecutable
        self.paramsFile = paramsFile

    def __repr__(self):
        return f"EventLaunchDescription({self.eventExecutable}, {self.paramsFile if self.paramsFile else '<Default Params>'})"

    def toRosMessage(self):
        file_contents = None
        if self.paramsFile:
            with open(self.paramsFile, "r") as file:
                file_contents = file.read()

        msgData = {"eventExecutable": self.eventExecutable, "params_file_name": self.paramsFile.split("/")[-1] if self.paramsFile else None, "params": file_contents}
        msg = String()
        msg.data = json.dumps(msgData)

        return msg

    @staticmethod
    def fromRosMessage(message):
        data = json.loads(message.data)
        if data["params_file_name"]:
            create_directory_if_not_exists("/uploadedParams/")
            file_path = "/uploadedParams/" + data["params_file_name"]
            with open(file_path, "w") as file:
                file.write(data["params"])
        else:
            file_path = None
        return EventLaunchDescription(data["eventExecutable"], file_path)


class ImuData:
    def __init__(self, qx, qy, qz, qw=None):
        # yaw, pitch, roll is using euler
        self.qx = float(qx)
        self.qy = float(qy)
        self.qz = float(qz)

        if qw:
            self.qw = float(qw)
            self.yaw, self.pitch, self.roll = quaternion_to_euler(self.qx, self.qy, self.qz, self.qw)
        else:
            self.yaw, self.pitch, self.roll = self.qx, self.qy, self.qz

    def __repr__(self):
        return f"IMU(pitch: {self.pitch}, roll: {self.roll}, yaw: {self.yaw})"

    def toRosMessage(self):
        msgData = {
            "yaw": self.yaw,
            "pitch": self.pitch,
            "roll": self.roll,
        }
        msg = String()
        msg.data = json.dumps(msgData)

        return msg

    @staticmethod
    def fromRosMessage(message):
        data = json.loads(message.data)
        return ImuData(data["yaw"], data["pitch"], data["roll"])


class UsbResetCmd:
    def __init__(self, hub, port):
        # yaw, pitch, roll is using euler
        self.hub = hub
        self.port = port

    def toRosMessage(self):
        msgData = {"hub": self.hub, "port": self.port}
        msg = String()
        msg.data = json.dumps(msgData)

        return msg

    @staticmethod
    def fromRosMessage(message):
        data = json.loads(message.data)
        return UsbResetCmd(data["hub"], data["port"])


def create_directory_if_not_exists(file_path):
    directory = os.path.dirname(file_path)
    if not os.path.exists(directory):
        os.makedirs(directory)


# TODO: make function use ROS to resolve circulat import error
def has_reached_waypoint(waypoint, distance=float(c.config["CONSTANTS"]["reached_waypoint_distance"])):
    """Returns true/false if the boat is close enough to the waypoint"""
    # a = GPS()
    # boat_gps = Waypoint(a.latitude, a.longitude)
    # return distance_between(boat_gps, waypoint) < distance
    return None


def ros_spin_some(node, executor=None, timeout_sec=0, wait_condition=lambda: False):
    """
    execute ros callbacks until there are no more available or for timeout_sec, whichever comes first
    if timeout_sec is 0 then it will execute callbacks until there are no more available
    """
    if timeout_sec != 0:
        endTime = datetime.now() + timeout_sec
    executor = rclpy.get_global_executor() if executor is None else executor
    executor.add_node(node)
    while True:
        if timeout_sec != 0:
            remainingTime = endTime - datetime.now()
            if remainingTime <= 0:
                break
        else:
            remainingTime = 0.0

        try:
            handler, _, _ = executor.wait_for_ready_callbacks(remainingTime, None, wait_condition)
        except ShutdownException:
            pass
        except TimeoutException:
            break
        else:
            handler()
            if handler.exception() is not None:
                raise handler.exception()

    executor.remove_node(node)
