"""
Reads and sends data from the connected USB transceiver
"""
import time
import os

# from messages_pb2 import *
import rclpy
import serial
import smbus2 as smbus  # ,smbus2
from rclpy.node import Node
from std_msgs.msg import String
from dataclasses import dataclass

from sailbot import constants as c

I2C_SLAVE_ADDRESS = 0x10


class Transceiver(Node):
    """Handles all communication between the boat and shore
    Functions:
        send(): sends a string to the to the transmitter
        read(): reads the state of the RC controller
    """

    def __init__(self):
        super().__init__("transceiver")
        self.logging = self.get_logger()

        self.pub = self.create_publisher(String, "transceiver", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        ports = [c.config["TRANSCEIVER"]["ardu_port"], c.config["TRANSCEIVER"]["ardu_port2"], c.config["TRANSCEIVER"]["ardu_port3"]]
        for i, port in enumerate(ports):
            try:
                # High timeout (5s+) is necessary to prevent falsely flagging a port as invalid due to initialization time
                # May cause runtime latency tho if not threaded and the transceiver arduino code isn't writing anything to serial
                self.ser = serial.Serial(port, int(c.config["TRANSCEIVER"]["baudrate"]), timeout=5, exclusive=False)

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

        # self.sail_publisher = self.create_publisher(String, "transceiver", 10)
        # self.rudder_publisher = self.create_publisher(String, "transceiver", 10)

    def timer_callback(self):
        msg = String()
        msg.data = self.readData()
        self.logging.info('Publishing: "%s"' % msg.data)
        self.pub.publish(msg)

    def send(self, data):
        self.ser.write(str(data).encode())

    def read(self) -> str or None:
        """Reads incoming data from the RC controller"""
        message = self.ser.readline().decode().replace("\r\n", "").replace("b'", "").replace("\\r\\n'", "")

        if message is None or message == "'" or message == "":
            return None

        self.logging.debug(f"Received message {message}")

        return message

    def readData(self):
        """Reads rudder/sail position"""
        self.send("?")  # transceiver is programmed to respond to '?' with its data

        msg = self.read()

        if msg is None:
            time.sleep(5)
            msg = self.read()
            if msg is None:
                raise RuntimeError("Failed to read from transceiver")

        splits = msg.split(" ")
        if len(splits) >= 7:
            returnList = [F"{splits[0]} {splits[1]}",
                          F"{splits[2]} {splits[3]}"]  # rudderOffset (val), sailOffset (val)

            mode, offset = GetModeAndOffset(float(splits[5]), float(splits[7]))

            returnList.append(F"{mode} {offset}")

            return str(returnList)
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


@dataclass
class Controller:
    """
    Stores the current state of the RC controller as read from the transceiver
        - Analog and potentiometers range from 0-100
        - Switches can be 0 (down), 1 (center), or 2 (up)
    """

    left_analog_x: int
    left_analog_y: int  # Sail
    right_analog_x: int  # Rudder
    right_analog_y: int
    front_left_switch: int  # Offset mode (0 - Rudder, 2 - Sail)
    left_potentiometer: int  # Offset
    front_right_switch: int  # Autonomy (0 - Autonomous, 2 - RC)
    top_left_switch: int
    top_right_switch: int


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/transceiver"
    rclpy.init(args=args)
    transceiver = Transceiver()
    rclpy.spin(transceiver)


if __name__ == "__main__":
    main()
