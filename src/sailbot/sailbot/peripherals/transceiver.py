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

        if message is None or message == "":
            return

        self.logging.info(f"Received message {message}")
        self.send("Received message")

        return str(message)


def ConvertStringsToBytes(src) -> list[int]:
    converted = []
    for b in src:
        converted.append(ord(b))
    return converted

# TODO: deprecate
class arduino:

    def __init__(self, port_num):
        # connect to device on 'port_num'
        self.ser1 = serial.Serial(port_num, c.config['MAIN']['baudrate'], timeout = .5) 
        self.I2Cbus = smbus.SMBus(1)

    def send(self, data):
        #print(data)
        self.ser1.write(str(data).encode())

    def readData(self):
        # get data from transceiver
        self.send("?") # transceiver is programmed to respond to '?' with its data
        msgs = []
        msg = self.read()
        if msg == None or msg == "'":
            time.sleep(.1)
            msg = self.read()
            if msg == None or msg == "'":
                self.send("?")
                msg = self.read()
        # print(msg)
        splits = msg.split(" ")
        if len(splits) >= 7:
            returnList = [F"{splits[0]} {splits[1]}", F"{splits[2]} {splits[3]}"] # rudderOffset (val), sailOffset (val)
        
            mode, offset = self.getModeAndOffset(float(splits[5]), float(splits[7]))

            returnList.append(F"{mode} {offset}")

            return returnList
        else:
            return msg
                
        

    def read(self):
        message = str(self.ser1.readline()).replace("\r\n'", "").replace("b'", "").replace("\\r\\n'", "")
        
        return str(message)

    def getModeAndOffset(self, readModeVal, readOffsetVal):
        modes = { # mappings of the read mode integer to the function that should be used with the offsetValue
            "controlOff" : 0,
            "sailOffset" : 45,
            "rudderOffset" : 90
        }

        bestMode = None
        bestVal = None
        for key, value in modes.items():
            if bestVal == None or abs(readModeVal - value) < bestVal:
                bestMode = key
                bestVal = abs(readModeVal - value)

        if bestMode == "sailOffset":
            offset = self.map(readOffsetVal, 0, 90, -2, 2)

        elif bestMode == "rudderOffset":
            offset = self.map(readOffsetVal, 0, 90, -0.2, 0.2)

        else:
            offset = 0

        return bestMode, offset

    def map(self, x, min1, max1, min2, max2):
        # converts value x, which ranges from min1-max1, to a corresponding value ranging from min2-max2
        # ex: map(0.3, 0, 1, 0, 100) returns 30
        # ex: map(70, 0, 100, 0, 1) returns .7
        x = min(max(x, min1), max1)
        return min2 + (max2-min2)*((x-min1)/(max1-min1))

if __name__ == "__main__":
    transceiver = Transceiver()

    time.sleep(1)
    while True:
        print(transceiver.readData())
