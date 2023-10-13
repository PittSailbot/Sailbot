import importlib
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sailbot import boatMovement
from sailbot.events import precisionNavigation, endurance, search, stationKeeping
from sailbot.utils.eventUtils import EventFinished
from sailbot.utils.utils import singleton, Waypoint

from sailbot.peripherals import windvane, GPS, compass, transceiver

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if DOCKER == "True" else False

# folder = "sailbot.peripherals." if not DOCKER else "sailbot.virtualPeripherals."
# windVane = importlib.import_module(folder + "windvane").WindVane
# gps = importlib.import_module(folder + "GPS").GPS
# compass = importlib.import_module(folder + "compass").Compass
# transceiver = importlib.import_module(folder + "transceiver").Transceiver


events = {
    "ED": endurance.Endurance,
    "PN": precisionNavigation.PrecisionNavigation,
    "SE": search.Search,
    "SK": stationKeeping.StationKeeping,
}


@singleton
class Boat(Node):
    """The mf'in boat.
    - Communicates with shore and listens for run-time commands
    - Moves towards event waypoints
    - Handles lower level exceptions and errors
    """

    def __init__(self, event=None, event_data=None):
        super().__init__("main")
        self.logging = self.get_logger()

        # BOAT STATE
        self.is_RC = True if event is None else False
        self.event = init_event(event, event_data) if not self.is_RC else None
        self.next_gps = None

        # SENSORS
        self.transceiver = transceiver.Transceiver()
        self.gps = GPS.GPS()
        self.compass = compass.Compass()
        self.windvane = windvane.WindVane()

        # Controls
        self.sail = boatMovement.Sail()
        self.rudder = boatMovement.Rudder()

    def __del__(self):
        self.logging.info("Shutting down")

    def main_loop(self):
        """
        The main control logic which runs each tick
        """

        command = self.transceiver.readData()
        self.execute_command(command)

        if self.is_RC:
            # TODO: read from controller
            self.sail.angle = 0
            self.rudder.angle = 0

        else:
            try:
                if self.event is not None:
                    self.next_gps = self.event.next_gps()

                if self.next_gps is not None:
                    boatMovement.go_to_gps(self.next_gps)

                else:
                    self.sail.angle = 0

            except EventFinished:
                self.logging.info("Event finished. Returning to RC")

                self.is_RC = True
                self.event = None

            except Exception as e:
                self.logging.critical(f"""Unhandled exception occured: {e}
                        Returning to RC!""")

                self.is_RC = True
                self.event = None

    def execute_command(self, cmds):
        """Executes commands given to the boat from the RC controller

        Args:
            - args (list): any of the following valid commands

        Valid Commands:
            - RC {on/off}: enable or disable RC
            - sail {float}: set the sail to a specific angle
            - rudder {float}: set the rudder to a specific angle
            - sailoffset {float}: set the offset for the sail
            - rudderoffset {float}: set the offset for the rudder
            - setevent {str} {args}: set a new event with specified arguments (INCOMPLETE)
            - goto {lat} {lon}: autonomously move the boat to the determined waypoint
        """
        if cmds is None or cmds == []:
            return

        # cmds formatted like ['R 43', 'S 0', 'sailOffset -0.3555555555555556']
        # ['R 45', 'S 0', 'controlOff 0']
        self.logging.debug(f"Received command: {cmds}")

        for cmd in cmds:
            # Format "R 45" -> ['r', '45']
            cmd = cmd.split()
            cmd = [arg.lower() for arg in cmd]

            try:
                if cmd[0] == "rc" or cmd[0] == "sailoffset" or cmd[0] == "rudderoffset":
                    if cmd[1] == "on":
                        self.logging.info("Enabled RC")
                        self.is_RC = True
                    elif cmd[1] == "off" or cmd[0] == "controloff":
                        self.logging.info("Disabled RC")
                        self.is_RC = False

                elif self.is_RC and cmd[0] == "s":
                    if self.is_RC:
                        self.logging.info(f"Adjusting sail to {float(cmd[1])}")
                        self.sail.angle = float(cmd[1])
                    else:
                        self.logging.info("Refuse to change sail, not in RC Mode")

                elif self.is_RC and cmd[0] == "r":
                    if self.is_RC:
                        self.logging.info(f"Adjusting rudder to {float(cmd[1])}")
                        self.rudder.angle = float(cmd[1])
                    else:
                        self.logging.info("Refuse to change sail, not in RC Mode")

                elif self.is_RC and cmd[0] == "sailoffset":
                    dataStr = String()
                    dataStr.data = f"(driverOffset:sail:{float(cmd[1])})"
                    self.logging.info(dataStr.data)
                    self.pub.publish(dataStr)

                elif self.is_RC and cmd[0] == "rudderoffset":
                    dataStr = String()
                    dataStr.data = f"(driverOffset:rudder:{float(cmd[1])})"
                    self.logging.info(dataStr.data)
                    self.pub.publish(dataStr)

                # elif cmd[0] == "setevent":
                    # if cmd[1] not in events:
                        # self.logging.info(f"Invalid event: {cmd[1]}")
                        # return

                    # self.event = init_event(cmds[1], cmds[2:])
                    # self.is_RC = False

                # elif cmd[0] == "goto":
                    # if not self.is_RC:
                        # target = Waypoint(float(cmd[1]), float(cmd[2]))
                        # self.logging.info(f"Going to: {target}")

                else:
                    self.logging.info(f"Unknown command: {cmd}")

            except IndexError as e:
                self.logging.info(f"Invalid length args for command: {cmd}\n{e}")

            except Exception as e:
                self.logging.info(f"Error when parsing command: {cmd}\n{e}")


def init_event(name, event_data):
    """Returns an initialized event using the event_data"""
    event = events[name]
    return event(event_data)


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/main"
    rclpy.init(args=args)

    # parser = argparse.ArgumentParser(prog="Sailbot")
    # parser.add_argument("-e", "--event", help="the mode that the boat starts in, default is RC", type=str)
    # parser.add_argument("-d","--debug",help="raises exceptions instead of ignoring them (so that the boat doens't crash during competition), default is off",)

    # args = parser.parse_args()

    boat = Boat()

    while True:
        try:
            boat.main_loop()
        except KeyboardInterrupt:
            print("Exiting gracefully.")
            break


if __name__ == "__main__":
    main()
