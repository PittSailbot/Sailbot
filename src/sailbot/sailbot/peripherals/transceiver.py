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
# https://www.geeksforgeeks.org/how-to-install-protocol-buffers-on-windows/
# Compile .proto with `protoc teensy.proto --python_out=./`
from sailbot.telemety.protobuf import teensy_pb2


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
        self.rudder_pub = self.create_publisher(String, "cmd_rudder", 1)

        self.sail_offset_pub = self.create_publisher(String, "offset_sail", 1)
        self.rudder_offset_pub = self.create_publisher(String, "offset_rudder", 1)
        self.prev_offset = 50

        self.wind_angle_pub = self.create_publisher(String, "wind_angle", 1)

    def timer_callback(self):
        """Publishes all data received from the teensy onto the relevant topics
        Read the string into a protobuf object using controlsData_pb2.ParseFromString(msg)"""
        teensy_data = teensy_pb2.ParseFromString(self.read())

        self.logging.info(f"Received {teensy_data}")

        self.publish_controller(teensy_data.controller)
        self.wind_angle_pub.publish(String(data=teensy_data.windvane.wind_angle))

    def send(self, data):
        self.ser.write(str(data).encode())

    def read(self):
        """Reads incoming data from the Teensy"""
        self.send("?")  # transceiver is programmed to respond to '?' with its data

        message = teensy_pb2.Data(self.ser.readline())

        return message

    def publish_controller(self, controller: teensy_pb2.Controller):
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
            self.sail_pub.publish(String(0))
            self.rudder_pub.publish(String(50))
            return

        if RC_ENABLED:
            self.rc_enabled_pub.publish(String("1"))
            OFFSET_MODE = controller.front_right_switch
            AUTO_SET_SAIL = True if controller.front_left_switch1 == 2 else False

            if AUTO_SET_SAIL:
                pass
                # TODO: auto set sail navigation.auto_adjust_sail()
            else:
                self.sail_pub.publish(String(data=controller.left_analog_y))
            self.rudder_pub.publish(String(data=controller.right_analog_x))

            if OFFSET_MODE != 0:
                relative_offset = controller.potentiometer - self.prev_offset
                if OFFSET_MODE == 1:
                    self.sail_offset_pub.publish(String(relative_offset))
                elif OFFSET_MODE == 2:
                    self.rudder_offset_pub.publish(String(relative_offset))
            else:
                self.prev_offset = controller.potentiometer
        else:
            self.rc_enabled_pub.publish(String(""))


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/transceiver"
    rclpy.init(args=args)
    transceiver = Transceiver()
    rclpy.spin(transceiver)


if __name__ == "__main__":
    main()
