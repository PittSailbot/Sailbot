"""
Brokers messages between ROS2 and the microcontroller using protobuf over USB Serial
- Messages incoming from mcu are converted into ROS2 messages and republished
- Messages going to the mcu are converted into protobuf
"""

import os
import time
from time import sleep

import rclpy
import serial
import smbus2 as smbus
from geometry_msgs.msg import Quaternion
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from serial.tools import list_ports
from std_msgs.msg import Float32, Int32, String

from sailbot import constants as c
from sailbot.protobuf import mcu_pb2, pi_pb2
from sailbot.utils import boatMath
from sailbot.utils.utils import ControlState, ImuData, Waypoint

# from geographic_msgs.msg import GeoPose, GeoPoint


class MCUBridge(Node):
    """Handles communication between the microcontroller and pi.
    Functions:
        send(): sends a string to the to the microcontroller
        read(): reads the state of the Teensy

    Publishes to:
        - /sail
        - /jib
        - /rudder
        - /navigation TODO: Autonomy ON/OFF
        - /wind_angle
        - /position
        - /speed

    Subscribes to:
        - /cmd_sail
        - /cmd_jib
        - /cmd_rudder
    """

    def __init__(self):
        super().__init__("transceiver")
        self.logging = self.get_logger()

        self._frame_magic = bytes([0xA5, 0x5A])
        self._frame_header_len = 4
        self._max_payload_len = 4096
        self._rx_buffer = bytearray()

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.sail_offset_last_message_value = 0
        self.rudder_offset_last_message_value = 0
        self.last_motor_offset_state = 0

        self.ser = None
        error = self.setup_com()
        if error:
            raise error

        # self.I2Cbus = smbus.SMBus(1)

        # self.controller_pub = self.create_publisher(String, "controller_state", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.control_state_pub = self.create_publisher(String, "control_state", 1)

        self.sail_pub = self.create_publisher(Float32, "sail", 1)
        self.jib_pub = self.create_publisher(Float32, "jib", 1)
        self.rudder_pub = self.create_publisher(Float32, "rudder", 1)

        self.sail_offset_pub = self.create_publisher(Float32, "offset_sail", 1)
        self.rudder_offset_pub = self.create_publisher(Float32, "offset_rudder", 1)

        self.wind_angle_pub = self.create_publisher(String, "wind_angle", 1)

        # self.position_pub = self.create_publisher(String, "position", 1)
        # self.speed_pub = self.create_publisher(String, "speed", 1)

        self.imu_pub = self.create_publisher(String, "imu", 1)
        self.compass_offset_sub = self.create_subscription(Float32, "/offset_compass", self.compass_offset_callback, 1)
        self.compass_offset = 0

        self.usbReset_pub = self.create_publisher(String, "usbReset", 1)

        self.event_control_sub = self.create_subscription(Int32, "/event_control_state", self.event_control_state_callback, 1)
        self.event_control_state = ControlState.AUTO

        self.cmd_sail_sub = self.create_subscription(Float32, "/cmd_sail", self.cmd_sail_callback, 1)
        self.cmd_jib_sub = self.create_subscription(Float32, "/cmd_jib", self.cmd_jib_callback, 1)
        self.cmd_rudder_sub = self.create_subscription(Float32, "/cmd_rudder", self.cmd_rudder_callback, 1)

    def compass_offset_callback(self, msg):
        self.compass_offset = float(msg.data)

    def event_control_state_callback(self, msg):
        self.event_control_state = msg.data

    def cmd_sail_callback(self, msg: Float32):
        self.send_cmd(cmd_sail=msg.data)

    def cmd_jib_callback(self, msg: Float32):
        self.send_cmd(cmd_jib=msg.data)

    def cmd_rudder_callback(self, msg: Float32):
        self.send_cmd(cmd_rudder=msg.data)

    def _log_mcu_text_line(self, line: bytes):
        text = line.decode("utf-8", errors="replace").strip()
        if not text:
            return

        prefix = text[:2]
        if prefix == "E:":
            self.logging.error(f"MCU: {text}")
        elif prefix == "W:":
            self.logging.warning(f"MCU: {text}")
        elif prefix in ("D:", "V:"):
            self.logging.debug(f"MCU: {text}")
        else:
            self.logging.info(f"MCU: {text}")

    def setup_com(self, port_description=str(c.config["MCU_BRIDGE"]["usb_mcu_name"])):
        """Initializes a Serial connection on the USB device matching the desired name"""
        found_ports, found_descriptions, found_hwids = self.list_ports()

        if len(found_ports) == 0:
            raise Exception("No connected devices found. If this is in WSL2 you must attach the USB device with usbipd & reboot the container.")

        for i, port in enumerate(found_ports):
            if not (port_description.strip().lower().replace('"', "").replace("'", "")) in str(found_descriptions[i]).strip().lower().replace('"', "").replace("'", ""):
                if i == len(found_ports) - 1:
                    debug_str = f"Failed to find device matching name: '{c.config['MCU_BRIDGE']['usb_mcu_name']}'\n"
                    debug_str += "Found ports are: \n"
                    for i in range(len(found_ports)):
                        debug_str += f"\tPort: [{found_ports[i]}], Name: [{found_descriptions[i]}] HWID: [{found_hwids[i]}]\n"
                    self.logging.fatal(debug_str)
                    return RuntimeError("Failed to find any matching USB devices.")
                else:
                    continue
            try:
                self.logging.debug("Found USB device HWID:" + str(found_hwids[i]))
                # High timeout (5s+) is necessary to prevent falsely flagging a port as invalid due to initialization time
                # May cause runtime latency if not threaded and the microcontroller arduino code isn't writing anything to serial
                self.ser = serial.Serial(port, int(c.config["MCU_BRIDGE"]["baudrate"]), timeout=5, exclusive=False)
                self.ser.reset_input_buffer()
                self.last_successful_message = time.time()

            except Exception as e:
                self.logging.error(f"Failed to read from port: {port}. No data sent.")
                return e

            else:
                self.logging.info(f"Serial initialized with port: {port}")
                return

    def timer_callback(self):
        """Publishes all data received from the mcu onto the relevant topics
        Read the string into a protobuf object using controlsData_pb2.ParseFromString(msg)"""
        try:
            teensy_data = self.read()
        except Exception as e:
            self.logging.error(f"Error while reading from mcu: {e}")
            self.timer.cancel()
            self.setup_com()
            sleep(1)
            self.timer.reset()
            return

        self.logging.debug(str(teensy_data))

        if teensy_data is None:
            return

        if teensy_data.HasField("rc_data"):
            self.publish_controller(teensy_data.rc_data)
        # else:
        # self.logging.warning("No RC data", throttle_duration_sec=60)

        if teensy_data.HasField("windvane"):
            self.wind_angle_pub.publish(String(data=str(teensy_data.windvane.wind_angle)))

        if teensy_data.HasField("gps"):
            self.gps_pub.publish(Waypoint(teensy_data.gps.lat, teensy_data.gps.lon).to_msg())
            msg = String()
            msg.data = str(teensy_data.gps.speed)
            self.speed_pub.publish(msg)

        if teensy_data.HasField("imu"):
            imu = teensy_data.imu
            msg = ImuData((imu.yaw + self.compass_offset) % 360, imu.pitch, imu.roll).toRosMessage()
            self.imu_pub.publish(msg)
            self.logging.debug('Publishing: "%s"' % msg.data)

    def send(self, payload: bytes):
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("Serial connection is not open")

        payload_len = len(payload)
        if payload_len <= 0 or payload_len > self._max_payload_len:
            raise ValueError(f"Invalid payload length: {payload_len}")

        header = self._frame_magic + payload_len.to_bytes(2, byteorder="little", signed=False)
        self.ser.write(header)
        self.ser.write(payload)

    def _to_servo_percent(self, value: float) -> int:
        # Keep command values within the expected -100..100 percentage range.
        return max(0, min(100, int(round(value))))

    def send_cmd(self, cmd_sail: float = None, cmd_jib: float = None, cmd_rudder: float = None):
        pi_msg = pi_pb2.PiData()

        if cmd_sail is not None:
            pi_msg.cmd_sail = self._to_servo_percent(cmd_sail)
        if cmd_jib is not None:
            pi_msg.cmd_jib = self._to_servo_percent(cmd_jib)
        if cmd_rudder is not None:
            pi_msg.cmd_rudder = int(cmd_rudder)

        if not pi_msg.ListFields():
            return

        try:
            self.logging.debug(f"Sending PiData to MCU: {pi_msg}")
            self.send(pi_msg.SerializeToString())
        except Exception as e:
            self.logging.error(f"Failed to send command protobuf to mcu: {e}")

    def read(self):
        """Reads framed incoming protobuf data from the MCU.

        Frame format:
            [0xA5, 0x5A, payload_len_lo, payload_len_hi, payload...]
        """
        if self.ser.in_waiting > 0:
            incoming = self.ser.read(self.ser.in_waiting)
            if incoming:
                self._rx_buffer.extend(incoming)
                self.last_successful_message = time.time()

        while self._rx_buffer:
            start = self._rx_buffer.find(self._frame_magic)
            newline = self._rx_buffer.find(b"\n")

            if start < 0:
                if newline >= 0:
                    line = bytes(self._rx_buffer[: newline + 1])
                    del self._rx_buffer[: newline + 1]
                    self._log_mcu_text_line(line)
                    self.last_successful_message = time.time()
                    continue

                if len(self._rx_buffer) > 4096:
                    # Drop unexpectedly long unframed data instead of letting the buffer grow forever.
                    self._log_mcu_text_line(bytes(self._rx_buffer))
                    self._rx_buffer.clear()
                break

            if newline >= 0 and newline < start:
                line = bytes(self._rx_buffer[: newline + 1])
                del self._rx_buffer[: newline + 1]
                self._log_mcu_text_line(line)
                self.last_successful_message = time.time()
                continue

            if start > 0:
                text = bytes(self._rx_buffer[:start])
                del self._rx_buffer[:start]
                self._log_mcu_text_line(text)
                self.last_successful_message = time.time()

            if len(self._rx_buffer) < self._frame_header_len:
                break

            payload_len = self._rx_buffer[2] | (self._rx_buffer[3] << 8)
            if payload_len <= 0 or payload_len > self._max_payload_len:
                # Invalid header; resync at next byte.
                del self._rx_buffer[0]
                continue

            frame_len = self._frame_header_len + payload_len
            if len(self._rx_buffer) < frame_len:
                # Need more bytes.
                break

            payload = bytes(self._rx_buffer[self._frame_header_len : frame_len])
            del self._rx_buffer[:frame_len]

            try:
                message = mcu_pb2.TeensyData()
                message.ParseFromString(payload)
                if str(message).strip() != "":
                    self.last_successful_message = time.time()
                    return message
            except Exception:
                # Ignore malformed frame and continue scanning.
                continue

        if (time.time() - self.last_successful_message) > 10:
            self.usbReset_pub.publish(String(data=""))
            self.logging.error("Resetting usb", throttle_duration_sec=1)

        if (time.time() - self.last_successful_message) > 3:  # seconds
            self.logging.error("No valid message recived in awhile, check microcontroller", throttle_duration_sec=10)
        return None

    def publish_controller(self, controller: mcu_pb2.RCData):
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
    ros_log_base = os.getenv("ROS_LOG_DIR_BASE", "/tmp/ros_logs")
    os.environ["ROS_LOG_DIR"] = ros_log_base + "/mcu_bridge"
    rclpy.init(args=args)
    mcu_bridge = MCUBridge()
    rclpy.spin(mcu_bridge)


if __name__ == "__main__":
    main()
