"""
Reads and sends data from the connected USB transceiver
"""
import time
import os

from sailbot.telemetry.protobuf import controlsData_pb2, teensy_pb2
import rclpy
import serial
from serial.tools import list_ports
import smbus2 as smbus
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Quaternion
# from geographic_msgs.msg import GeoPose, GeoPoint

from sailbot import constants as c
# https://www.geeksforgeeks.org/how-to-install-protocol-buffers-on-windows/
# Compile .proto with `protoc teensy.proto --python_out=./`
from sailbot.utils.utils import Waypoint


class Transceiver(Node):
    """Handles all communication between the boat and shore. Also publishes all sensors on the Teensy.
    Functions:
        send(): sends a string to the to the transmitter
        read(): reads the state of the Teensy

    Publishes to:
        - /cmd_sail
        - /offset_sail
        - /cmd_rudder
        - /offset_rudder
        - /navigation TODO: Autonomy ON/OFF
        - /wind_angle
        - /position
        - /speed
    """

    def __init__(self):
        super().__init__("transceiver")
        self.logging = self.get_logger()

        self.pub = self.create_publisher(String, "transceiver", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        found_ports, found_descriptions, found_hwids = self.listPorts()

        for i, port in enumerate(found_ports):
            if str(found_hwids[i]).strip().lower().replace('"', ''). replace("'", "") != (str(c.config["TRANSCEIVER"]["transceiver_hwid"]).strip().lower().replace('"', ''). replace("'", "")):

                if i == len(found_ports) - 1:
                    self.logging.fatal("Failed to read from all transceiver ports! Is the transceiver plugged in?")
                    debug_str = "Found ports are: \n"
                    for i in range(len(found_ports)):
                        debug_str += F"\tPort: [{found_ports[i]}], Name: [{found_descriptions[i]}] HWID: [{found_hwids[i]}]\n"
                    debug_str += F'Failed to find HWID: {c.config["TRANSCEIVER"]["transceiver_hwid"]}\n'
                    self.logging.warning(debug_str)
                    raise RuntimeError("Failed to read from all transceiver ports! Is the transceiver plugged in?")
                else:
                    continue
            try:
                # High timeout (5s+) is necessary to prevent falsely flagging a port as invalid due to initialization time
                # May cause runtime latency if not threaded and the transceiver arduino code isn't writing anything to serial
                self.ser = serial.Serial(port, int(c.config["TRANSCEIVER"]["baudrate"]), timeout=5, exclusive=False)

                assert self.readRaw() is not None
                self.last_successful_message = time.time()

            except Exception as e:
                self.logging.error(f"Failed to read from port: {port}")
                self.logging.fatal("Failed to read from all transceiver ports! Is the transceiver plugged in?")
                raise RuntimeError("Failed to read from all transceiver ports! Is the transceiver plugged in?")

            else:
                self.logging.info(f"Transceiver initialized with port: {port}")
                break

        self.I2Cbus = smbus.SMBus(1)

        # self.controller_pub = self.create_publisher(String, "controller_state", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.rc_enabled_pub = self.create_publisher(String, "rc_enabled", 1)

        self.sail_pub = self.create_publisher(Float32, "cmd_sail", 1)
        self.rudder_pub = self.create_publisher(Float32, "cmd_rudder", 1)

        self.sail_offset_pub = self.create_publisher(String, "offset_sail", 1)
        self.rudder_offset_pub = self.create_publisher(String, "offset_rudder", 1)
        self.prev_offset = 50

        self.wind_angle_pub = self.create_publisher(String, "wind_angle", 1)

        self.position_pub = self.create_publisher(String, "position", 1)
        self.speed_pub = self.create_publisher(String, "speed", 1)

    def timer_callback(self):
        """Publishes all data received from the teensy onto the relevant topics
        Read the string into a protobuf object using controlsData_pb2.ParseFromString(msg)"""
        # TODO: try except to echo published non-protobuf error strings from Teensy
        teensy_data = self.read()

        if teensy_data == None:
            return

        self.logging.info(f"Received {teensy_data}")

        self.publish_controller(teensy_data.rc_data)
        # TODO: if null, don't pub
        if teensy_data.HasField("windvane"):
            self.wind_angle_pub.publish(String(data=str(teensy_data.windvane.wind_angle)))
        # TODO: Fuse imu and gps into geopose
        if teensy_data.HasField("gps"):
            self.gps_pub.publish(Waypoint(teensy_data.GPS.lat, teensy_data.GPS.lon).to_string())
            msg = String()
            msg.data = str(teensy_data.GPS.speed)
            self.speed_pub.publish(msg)

    def send(self, data):
        self.ser.write(str(data).encode())

    def read(self):
        """Reads incoming data from the Teensy"""
        # self.send("?")  # transceiver is programmed to respond to '?' with its data

        msg = self.ser.readline().strip()
        # self.logging.warning(msg)
        try:
            message = teensy_pb2.Data()
            retVal = message.ParseFromString(msg)
            # self.logging.warning("retVal:" + str(retVal))

            if str(message).strip() != '':
                self.last_successful_message = time.time()
                return message
        except Exception as e:
            # self.logging.warning(F"Exception: [{e}] raised when processing: {msg}")
            pass

        if (time.time() - self.last_successful_message) > 1: #seconds
            self.logging.warning("No valid message recived in awhile, check transceiver")
            self.last_successful_message = time.time()
        return None
        
    def readRaw(self):
        # self.send("?")  # transceiver is programmed to respond to '?' with its data

        return self.ser.readline()

    def publish_controller(self, controller: teensy_pb2.RCData):
        """Publishes the keybind/meaning of each controller input to the relevant topic.
        Editing this function will 'rebind' what an input does.

        left_analog_y       - Sail
        right_analog_x      - Rudder
        right_analog_y      -
        left_analog_x       -
        front_left_switch1  - Up (0): N/A     | Mid (1): None        | Down (2): Auto-set sail (RC)
        front_left_switch2  - Up (0):         | Mid (1):             | Down (2):
        front_right_switch  - Up (0): default | Mid (1): sail offset | Down (2): rudder offset
        top_left_switch     - Down (0): RC                           | Up (1): Autonomy
        top_right_switch    - TODO: Software reset (hold up 5s)  # Reset switch broken so disabled ;(
        potentiometer       - TODO: Sail/Rudder offsets
        """
        RC_ENABLED = True if controller.top_left_switch == 0 else False
        RESET_ENABLED = True if controller.top_right_switch == 1 else False

        if False and RESET_ENABLED:
            # TODO: wait 5s, zero out rudder & sail, then reboot
            ss = String()
            ss.data = controller.left_analog_y
            self.sail_pub.publish(ss)
            rs = String()
            ss.data = controller.right_analog_x
            self.rudder_pub.publish(rs)
            return

        if RC_ENABLED:
            rcMsg = String()
            rcMsg.data = "1"
            self.rc_enabled_pub.publish(rcMsg)
            OFFSET_MODE = controller.front_right_switch
            AUTO_SET_SAIL = True if controller.front_left_switch1 == 2 else False

            if AUTO_SET_SAIL:
                pass
                # TODO: auto set sail navigation.auto_adjust_sail()
            else:
                sailMsg = Float32()
                sailMsg.data = float(controller.left_analog_y)
                self.sail_pub.publish(sailMsg)
            rudderMsg = Float32()
            rudderMsg.data = float(controller.right_analog_x)
            self.rudder_pub.publish(rudderMsg)

            if OFFSET_MODE != 0:
                relative_offset = controller.potentiometer - self.prev_offset
                if OFFSET_MODE == 1:
                    sailOffsetMsg = String()
                    sailOffsetMsg.data = str(relative_offset)
                    self.sail_offset_pub.publish(sailOffsetMsg)
                elif OFFSET_MODE == 2:
                    rudderOffsetMsg = String()
                    rudderOffsetMsg.data = str(relative_offset)
                    self.rudder_offset_pub.publish(rudderOffsetMsg)
            else:
                self.prev_offset = controller.potentiometer
        else:
            rcMsg = String()
            rcMsg.data = ""
            self.rc_enabled_pub.publish(rcMsg)

    def listPorts(self):
        """!
        @brief Provide a list of names of serial ports that can be opened
        @return A tuple of the port list and a corresponding list of device descriptions, and hwids
        """
        ports = list( list_ports.comports() )

        resultPorts = []
        descriptions = []
        hwids = []
        for port in ports:
            # if port.device:
            resultPorts.append( port.device )
            descriptions.append( str( port.description ) )
            hwids.append( str( port.hwid ) )

        return (resultPorts, descriptions, hwids)


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/transceiver"
    rclpy.init(args=args)
    transceiver = Transceiver()
    rclpy.spin(transceiver)


if __name__ == "__main__":
    main()
