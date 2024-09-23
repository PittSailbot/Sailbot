"""
Reads and sends data from the connected USB transceiver
"""
import time
import os

from sailbot.protobuf import teensy_pb2
from sailbot.utils import boatMath
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
import serial
from serial.tools import list_ports
import smbus2 as smbus
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32
from geometry_msgs.msg import Quaternion
from time import sleep

# from geographic_msgs.msg import GeoPose, GeoPoint

from sailbot import constants as c
from sailbot.utils.utils import Waypoint, ControlState, ImuData


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

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.sail_offset_last_message_value = None
        self.rudder_offset_last_message_value = None
        self.last_motor_offset_state = None

        self.ser = None
        error = self.setup_coms()
        if error:
            raise error

        # self.I2Cbus = smbus.SMBus(1)

        # self.controller_pub = self.create_publisher(String, "controller_state", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.control_state_pub = self.create_publisher(String, "control_state", 1)

        self.sail_pub = self.create_publisher(Float32, "cmd_sail", 1)
        self.rudder_pub = self.create_publisher(Float32, "cmd_rudder", 1)

        self.sail_offset_pub = self.create_publisher(Float32, "offset_sail", 1)
        self.rudder_offset_pub = self.create_publisher(Float32, "offset_rudder", 1)

        self.wind_angle_pub = self.create_publisher(String, "wind_angle", 1)

        # self.position_pub = self.create_publisher(String, "position", 1)
        # self.speed_pub = self.create_publisher(String, "speed", 1)

        self.imu_pub = self.create_publisher(String, "imu", 1)
        self.compass_offset_sub = self.create_subscription(Float32, "/boat/offset_compass", self.compass_offset_callback, 1)
        self.compass_offset = 0

        self.usbReset_pub = self.create_publisher(String, "usbReset", 1)

        self.event_control_sub = self.create_subscription(Int32, "/boat/event_control_state", self.event_control_state_callback, 1)
        self.event_control_state = ControlState.AUTO

    def compass_offset_callback(self, msg):
        self.compass_offset = float(msg.data)

    def event_control_state_callback(self, msg):
        self.event_control_state = msg.data

    def setup_coms(self):
        found_ports, found_descriptions, found_hwids = self.list_ports()

        if len(found_ports) == 0:
            raise Exception("No connected devices found")

        for i, port in enumerate(found_ports):
            if not (str(c.config["TRANSCEIVER"]["transceiver_hwid"]).strip().lower().replace('"', '').replace("'", "")) in str(found_hwids[i]).strip().lower().replace('"', '').replace("'", ""):
                self.logging.info(str(found_hwids[i]))
                if i == len(found_ports) - 1:
                    self.logging.fatal("Failed to read from all transceiver ports! Is the transceiver plugged in?")
                    debug_str = "Found ports are: \n"
                    for i in range(len(found_ports)):
                        debug_str += F"\tPort: [{found_ports[i]}], Name: [{found_descriptions[i]}] HWID: [{found_hwids[i]}]\n"
                    debug_str += F'Failed to find HWID: {c.config["TRANSCEIVER"]["transceiver_hwid"]}\n'
                    self.logging.warning(debug_str)
                    return RuntimeError("Failed to read from all transceiver ports! Is the transceiver plugged in?")
                else:
                    continue
            try:
                self.logging.info("match " + str(found_hwids[i]))
                # High timeout (5s+) is necessary to prevent falsely flagging a port as invalid due to initialization time
                # May cause runtime latency if not threaded and the transceiver arduino code isn't writing anything to serial
                self.ser = serial.Serial(port, int(c.config["TRANSCEIVER"]["baudrate"]), timeout=5, exclusive=False)

                assert self.ser.readline() is not None
                self.logging.info("ser setup")
                self.last_successful_message = time.time()

            except Exception as e:
                self.logging.error(f"Failed to read from port: {port}")
                self.logging.fatal("Failed to read from all transceiver ports! Is the transceiver plugged in?")
                return e

            else:
                self.logging.info(f"Transceiver initialized with port: {port}")
                return

        raise Exception("Should be unreachable")

    def timer_callback(self):
        """Publishes all data received from the teensy onto the relevant topics
        Read the string into a protobuf object using controlsData_pb2.ParseFromString(msg)"""
        # TODO: try except to echo published non-protobuf error strings from Teensy
        try:
            teensy_data = self.read()
        except Exception as e:
            self.logging.error(F"Error while reading teensy: {e}")
            self.timer.cancel()
            self.setup_coms()
            sleep(1)
            self.timer.reset()
            return

        self.logging.debug(str(teensy_data))

        if teensy_data is None:
            return

        if teensy_data.HasField("rc_data"):
            self.publish_controller(teensy_data.rc_data)
        else:
            self.logging.warning("No RC data", throttle_duration_sec=3)

        if teensy_data.HasField("windvane"):
            self.wind_angle_pub.publish(String(data=str(teensy_data.windvane.wind_angle)))

        # if teensy_data.HasField("gps"):
        #     self.gps_pub.publish(Waypoint(teensy_data.gps.lat, teensy_data.gps.lon).to_msg())
        #     msg = String()
        #     msg.data = str(teensy_data.gps.speed)
        #     self.speed_pub.publish(msg)

        if teensy_data.HasField("imu"):
            imu = teensy_data.imu
            msg = ImuData((imu.yaw + self.compass_offset) % 360, imu.pitch, imu.roll).toRosMessage()
            self.imu_pub.publish(msg)
            self.logging.debug('Publishing: "%s"' % msg.data)

    def send(self, data):
        self.ser.write(str(data).encode())

    def read(self):
        """Reads incoming data from the Teensy"""
        msg = self.ser.readline().strip()
        if self.ser.in_waiting > 0:
            # empty queue
            _ = self.ser.read(self.ser.in_waiting)
        # self.logging.warning(msg)
        try:
            message = teensy_pb2.Data()
            ret_val = message.ParseFromString(msg)
            # self.logging.warning("ret_val:" + str(ret_val))

            if str(message).strip() != '':
                self.last_successful_message = time.time()
                return message
        except Exception as e:
            # self.logging.warning(F"Exception: [{e}] raised when processing: {msg}")
            pass

        if (time.time() - self.last_successful_message) > 10:
            self.usbReset_pub.publish(String(data=""))
            self.logging.error("Resetting transceiver", throttle_duration_sec=1)

        if (time.time() - self.last_successful_message) > 1:  # seconds
            self.logging.error("No valid message recived in awhile, check transceiver", throttle_duration_sec=1)
        return None

    def publish_controller(self, controller: teensy_pb2.RCData):
        """Publishes the keybind/meaning of each controller input to the relevant topic.
        Editing this function will 'rebind' what an input does.

        left_analog_y       - Sail
        right_analog_x      - Rudder
        right_analog_y      -
        left_analog_x       -
        front_left_switch1  - Up (0): Manual Sail/Rudder      | Mid (1): Manual Rudder  | Down (2): Autonomous
        front_left_switch2  - Up (0):                         | Mid (1):                | Down (2):
        front_right_switch  - Up (0): sail offset             | Mid (1): None           | Down (2): rudder offset
        top_left_switch     - Down (0): RC                    | Up (1): Autonomy
        top_right_switch    - TODO: Software reset (hold up 5s)  # Reset switch broken so disabled ;(
        potentiometer       - Sail/Rudder offsets
        """
        rudder_manual = ControlState.MANUAL if controller.front_left_switch1 <= 1 else self.event_control_state
        sail_manual = ControlState.MANUAL if controller.front_left_switch1 == 0 else self.event_control_state
        rcMsg = ControlState(rudder_manual, sail_manual).toRosMessage()
        self.control_state_pub.publish(rcMsg)

        motor_offset_mode = controller.front_right_switch

        if sail_manual == ControlState.MANUAL:
            sailMsg = Float32()
            sailMsg.data = float(controller.left_analog_y)
            self.sail_pub.publish(sailMsg)

            if motor_offset_mode == 0:
                if self.last_motor_offset_state == 0:
                    offsetChange = (controller.potentiometer - self.sail_offset_last_message_value) / 200

                    sailOffsetMsg = Float32()
                    sailOffsetMsg.data = float(offsetChange)
                    self.sail_offset_pub.publish(sailOffsetMsg)

                self.sail_offset_last_message_value = controller.potentiometer

        if rudder_manual == ControlState.MANUAL:
            rudderMsg = Float32()
            rudderMsg.data = float(controller.right_analog_x)
            self.rudder_pub.publish(rudderMsg)

            if motor_offset_mode == 2:
                if self.last_motor_offset_state == 2:
                    offsetChange = (controller.potentiometer - self.rudder_offset_last_message_value) / 200

                    rudderOffsetMsg = Float32()
                    rudderOffsetMsg.data = float(offsetChange)
                    self.rudder_offset_pub.publish(rudderOffsetMsg)

                self.rudder_offset_last_message_value = controller.potentiometer

        self.last_motor_offset_state = motor_offset_mode

    def list_ports(self):
        """Provides a list of names of serial ports that can be opened
        Returns:
            tuple: the port list and a corresponding list of device descriptions, and hwids
        """
        ports = list(list_ports.comports())

        resultPorts = []
        descriptions = []
        hwids = []
        for port in ports:
            # if port.device:
            resultPorts.append(port.device)
            descriptions.append(str(port.description))
            hwids.append(str(port.hwid))

        return resultPorts, descriptions, hwids


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/transceiver"
    rclpy.init(args=args)
    transceiver = Transceiver()
    rclpy.spin(transceiver)


if __name__ == "__main__":
    main()
