"""
Reads and sends data from the connected USB transceiver
"""
import os
import rclpy
import serial
import smbus2 as smbus
from rclpy.node import Node
from std_msgs.msg import String

from sailbot import constants as c
from sailbot.utils.utils import DummyObject
from sailbot.telemetry.protobuf import controlsData_pb2


class Transceiver(Node):
    """Handles all communication between the boat and shore
    Functions:
        send(): sends a string to the to the transmitter
        read(): reads the state of the RC controller

    Publishes to:
        - /controller_state TODO: Deprecate (transceiver.py become monolith) or move functionality to main
        - /cmd_sail
        - /offset_sail
        - /cmd_rudder
        - /offset_rudder
        - /navigation TODO: Autonomy ON/OFF
    """

    def __init__(self):
        super().__init__("transceiver")
        self.logging = self.get_logger()

        ports = [c.config["TRANSCEIVER"]["ardu_port"], c.config["TRANSCEIVER"]["ardu_port2"], c.config["TRANSCEIVER"]["ardu_port3"]]
        for i, port in enumerate(ports):
            try:
                # High timeout (5s+) is necessary to prevent falsely flagging a port as invalid due to initialization time
                # May cause runtime latency if not threaded and the transceiver arduino code isn't writing anything to serial
                self.ser = serial.Serial(port, int(c.config["TRANSCEIVER"]["baudrate"]), timeout=5, exclusive=False)

                assert self.read() is not None

            except OSError:
                self.logging.warning(f"No transceiver detected on port: {port}")
                if i == len(ports) - 1:
                    self.logging.fatal("Failed to read from all transceiver ports! Is the transceiver plugged in?")
                    raise RuntimeError("Failed to read from all transceiver ports! Is the transceiver plugged in?")

            except Exception as e:
                self.logging.error(f"Failed to read from port: {port}\nRaised: {e}")
                raise e

            else:
                self.logging.info(f"Transceiver initialized with port: {port}")
                break

        self.I2Cbus = smbus.SMBus(1)

        # self.controller_pub = self.create_publisher(String, "controller_state", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.rc_enabled_pub = self.create_publisher(String, "rc_enabled", 1)
        self.sail_pub = self.create_publisher(String, "cmd_sail", 1)
        self.sail_offset_pub = self.create_publisher(String, "offset_sail", 1)
        self.rudder_pub = self.create_publisher(String, "cmd_rudder", 1)
        self.rudder_offset_pub = self.create_publisher(String, "offset_rudder", 1)

    def timer_callback(self):
        """Publishes a binary representation of the RC controller's state.
        Read the string into a protobuf object using controlsData_pb2.ParseFromString(msg)"""
        controller_state = self.read()

        msg = String(data=str(controller_state.msg))
        self.logging.info(f"Publishing: {msg.data}")

        self.publish_control_signals(controller_state)

        # self.controller_pub.publish(msg)

    def send(self, data):
        self.ser.write(str(data).encode())

    def read(self):
        """Reads incoming data from the RC controller"""
        self.send("?")  # transceiver is programmed to respond to '?' with its data

        message = self.ser.readline()
        control_data = parse_serial(message)

        return control_data

    def publish_control_signals(self, controller_state):
        """Publishes the individual controls to each relevant topic"""
        RC_ENABLED = True if controller_state.top_left_switch == 0 else False
        RESET_ENABLED = True if controller_state.top_right_switch == 1 else False

        if RESET_ENABLED:
            # TODO: wait 5s, zero out rudder & sail, then reboot
            self.sail_pub.publish(String(0))
            self.rudder_pub.publish(String(50))
            return

        if RC_ENABLED:
            self.rc_enabled_pub.publish(String("1"))
            OFFSET_MODE = controller_state.front_right_switch
            AUTO_SET_SAIL = True if controller_state.front_left_switch1 == 2 else False

            if AUTO_SET_SAIL:
                pass
                # TODO: auto set sail navigation.auto_adjust_sail()
            else:
                self.sail_pub.publish(String(data=controller_state.left_analog_y))
            self.rudder_pub.publish(String(data=controller_state.right_analog_x))

            if OFFSET_MODE != 0:
                # TODO: set offsets as relative position of potentiometer
                offset = controller_state.potentiometer - 50
                if OFFSET_MODE == 1:
                    self.sail_offset_pub.publish(String(offset))
                elif OFFSET_MODE == 2:
                    self.rudder_offset_pub.publish(String(offset))
        else:
            self.rc_enabled_pub.publish(String(""))


def parse_serial(bytes):
    """Parses and unpacks RC controller state"""
    # Byte string is formatted as: b'50\t50\t65\t0\t0\t100\t100...\r\n'
    # TODO: Deprecate when Teensy is changed to protobuf
    values = bytes.split(b'\t')

    control_data = DummyObject()
    control_data.msg = values

    control_data.left_analog_y = str(values[0])       # Sail
    control_data.right_analog_x = str(values[1])      # Rudder
    control_data.right_analog_y = str(values[2])
    control_data.left_analog_x = str(values[3])
    control_data.front_left_switch1 = str(values[4])  # Up (0): N/A     | Mid (1): None        | Down (2): Auto-set sail (RC)
    control_data.front_left_switch2 = str(values[5])  # Up (0):         | Mid (1):             | Down (2):
    control_data.front_right_switch = str(values[6])  # Up (0): default | Mid (1): sail offset | Down (2): rudder offset
    control_data.top_left_switch = str(values[7])     # Down (0): RC                           | Up (1): Autonomy
    control_data.top_right_switch = str(values[8])    # Software reset (hold up 5s)
    control_data.potentiometer = str(values[9])       # Sail/Rudder offsets

    return control_data


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/transceiver"
    rclpy.init(args=args)
    transceiver = Transceiver()
    rclpy.spin(transceiver)


if __name__ == "__main__":
    main()
