"""
Reads and sends data from the connected USB transceiver
"""
import time

# from messages_pb2 import *
import rclpy
import serial
import smbus2 as smbus  # ,smbus2
from rclpy.node import Node
from std_msgs.msg import String

from sailbot import constants as c

I2C_SLAVE_ADDRESS = 0x10


class Transceiver(Node):
    """Handles all communication between the boat and shore
    Functions:
        send(): sends a string to the to shore
        read(): checks for any commands received from shore
    """

    def __init__(self):
        super().__init__("Transceiver")
        self.logging = self.get_logger()

        ports = [c.config["TRANSCEIVER"]["ardu_port"], c.config["TRANSCEIVER"]["ardu_port2"], c.config["TRANSCEIVER"]["ardu_port3"]]
        for i, port in enumerate(ports):
            try:
                # High timeout (5s+) is necessary to prevent falsely flagging a port as invalid due to initialization time
                # May cause runtime latency tho if not threaded and the transceiver arduino code isn't writing anything to serial
                self.ser1 = serial.Serial(port, int(c.config["TRANSCEIVER"]["baudrate"]), timeout=10, exclusive=False)

                self.readData()

            except Exception as e:
                self.logging.warning(f"Failed to read from port: {port}")
                # self.logging.warning(f"Raised {e}")

                if i == len(ports) - 1:
                    self.logging.fatal("Failed to read from all transceiver ports!")
                    raise RuntimeError("Failed to read from all transceiver ports")
            else:
                self.logging.info(f"Transceiver initialized with port: {port}")
                break

        self.I2Cbus = smbus.SMBus(1)

        # TODO: subscribe to nodes
        self.gps = None
        self.rudder_angle = None
        self.sail_angle = None
        self.compass_angle = None
        self.wind_speed = None
        self.wind_direction = None
        self.battery = None

    def __del__(self):
        self.ser1.close()

    def send(self, data):
        self.ser1.write(str(data).encode())

    def send_heartbeat(self):
        """Sends basic boat information in the format:
        [latitude, longitude, rudder_angle, sail_angle, compass_angle, wind_speed, wind_direction, battery]
        """
        data_str = (
            f"DATA: ({self.gps.latitude}, {self.gps.latitude}), {self.rudder_angle}, {self.sail_angle},"
            f" {self.compass_angle}, {self.wind_speed}, {self.wind_direction}, {self.battery}"
        )
        self.send(data_str)

    def read(self) -> str or None:
        """Reads incoming data from shore"""
        message = self.ser1.readline().decode().replace("\r\n", "").replace("b'", "").replace("\\r\\n'", "")

        if message is None or message == "'":
            return None

        self.logging.debug(f"Received message {message}")

        return message

    def readData(self):
        """Reads rudder/sail position"""
        self.send("?")  # transceiver is programmed to respond to '?' with its data

        msg = self.read()

        if msg is None or msg == "'":
            time.sleep(5)
            msg = self.read()
            if msg is None or msg == "'":
                raise RuntimeError("Failed to read from transceiver")

        splits = msg.split(" ")
        if len(splits) >= 7:
            returnList = [F"{splits[0]} {splits[1]}",
                          F"{splits[2]} {splits[3]}"]  # rudderOffset (val), sailOffset (val)

            mode, offset = GetModeAndOffset(float(splits[5]), float(splits[7]))

            returnList.append(F"{mode} {offset}")

            return returnList
        else:
            return msg


def ConvertStringsToBytes(src) -> list[int]:
    converted = []
    for b in src:
        converted.append(ord(b))
    return converted


def GetModeAndOffset(readModeVal, readOffsetVal):
    modes = { # mappings of the read mode integer to the function that should be used with the offsetValue
        "controlOff": 0,
        "sailOffset": 45,
        "rudderOffset": 90
    }

    bestMode = None
    bestVal = None
    for key, value in modes.items():
        if bestVal is None or abs(readModeVal - value) < bestVal:
            bestMode = key
            bestVal = abs(readModeVal - value)

    if bestMode == "sailOffset":
        offset = map(readOffsetVal, 0, 90, -2, 2)

    elif bestMode == "rudderOffset":
        offset = map(readOffsetVal, 0, 90, -0.2, 0.2)

    else:
        offset = 0

    return bestMode, offset


def map(x, min1, max1, min2, max2):
    # converts value x, which ranges from min1-max1, to a corresponding value ranging from min2-max2
    # ex: map(0.3, 0, 1, 0, 100) returns 30
    # ex: map(70, 0, 100, 0, 1) returns .7
    x = min(max(x, min1), max1)
    return min2 + (max2-min2)*((x-min1)/(max1-min1))


if __name__ == "__main__":
    rclpy.init()

    transceiver = Transceiver()

    while True:
        print(transceiver.readData())
