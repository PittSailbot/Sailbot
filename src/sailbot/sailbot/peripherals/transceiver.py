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

from src.sailbot.sailbot import constants as c

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

        ports = [c.config["MAIN"]["ardu_port"], c.config["MAIN"]["ardu_port2"], c.config["MAIN"]["ardu_port3"]]
        for i, port in enumerate(ports):
            try:
                self.ser1 = serial.Serial(port, int(c.config["MAIN"]["baudrate"]), timeout=0.5)
                self.send("?")
                assert self.read() is not None

            except Exception:
                self.logging.warning(f"Failed to read from port: {port}")

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
        message = self.ser1.readline().replace("\r\n'", "").replace("b'", "").replace("\\r\\n'", "")

        if message is None or message is "":
            return

        self.logging.info(f"Received message {message}")
        self.send("Received message")

        return str(message)


def ConvertStringsToBytes(src) -> list[int]:
    converted = []
    for b in src:
        converted.append(ord(b))
    return converted


if __name__ == "__main__":
    transceiver = Transceiver()

    time.sleep(1)
    while True:
        print(transceiver.readData())
