import importlib
import logging
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sailbot import boatMovement
from sailbot.events import precisionNavigation
from sailbot.events import endurance, search, stationKeeping
from sailbot.utils import eventUtils as eventUtils
from sailbot.utils.utils import singleton, Waypoint

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if DOCKER == "True" else False

folder = "sailbot.peripherals." if not DOCKER else "sailbot.virtualPeripherals."
windVane = importlib.import_module(folder + "windvane").WindVane
gps = importlib.import_module(folder + "GPS").GPS
compass = importlib.import_module(folder + "compass").Compass
transceiver = importlib.import_module(folder + "transceiver").Transceiver


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
        super().__init__("main_subscriber")
        self.logging = self.get_logger()

        # BOAT STATE
        self.is_RC = True if event is None else False
        self.event = init_event(event, event_data) if not self.is_RC else None
        self.next_gps = None

        # SENSORS
        self.transceiver = transceiver.Transceiver()
        self.gps = gps()
        self.compass = compass()
        self.windvane = windVane()

        # Controls
        self.sail = boatMovement.Sail()
        self.rudder = boatMovement.Rudder()

    def __del__(self):
        logging.info("Shutting down")
        transceiver.send("Shutting down")

    def main_loop(self):
        """
        The main control logic which runs each tick
        """

        command = self.transceiver.read()
        self.execute_command(command)

        if self.is_RC:
            # TODO: read from controller
            self.sail.angle = target_sail
            self.rudder.angle = target_rudder

        else:
            try:
                if self.event is not None:
                    self.next_gps = self.event.next_gps()

                if self.next_gps is not None:
                    boatMovement.go_to_gps(self.next_gps)

                else:
                    self.sail.angle = 0

            except eventUtils.EventFinished:
                msg = "Event finished. Returning to RC"
                self.logging.info(msg)
                self.transceiver.send(msg)

                self.is_RC = True
                self.event = None

            except Exception as e:
                msg = f"""Unhandled exception occured: {e}
                        Returning to RC!"""
                self.logging.critical(msg)
                self.transceiver.send(msg)

                self.is_RC = True
                self.event = None

    def execute_command(self, args):
        """Executes commands given to the boat from the transceiver

        Valid Commands:
        - RC {on/off}: enable or disable RC
        - sail {float}: set the sail to a specific angle
        - rudder {float}: set the rudder to a specific angle
        - sailoffset {float}: set the offset for the sail
        - rudderoffset {float}: set the offset for the rudder
        - setevent {str} {args}: set a new event with specified arguments (INCOMPLETE)
        - goto {lat} {lon}: autonomously move the boat to the determined waypoint
        """
        if args is None:
            return

        self.logging.debug(f"Received command: {args}")
        args = args.lower().split()

        try:
            if args[0] == "rc":
                if args[1] == "on":
                    self.transceiver.send("Enabled RC")
                    self.is_RC = True
                elif args[1] == "off":
                    self.transceiver.send("Disabled RC")
                    self.is_RC = False

            elif args[0] == "sail" or args[0] == "s":
                if self.is_RC:
                    self.transceiver.send(f"Adjusting sail to {float(args[1])}")
                    self.sail.angle = float(args[1])
                else:
                    self.transceiver.send("Refuse to change sail, not in RC Mode")

            elif args[0] == "rudder" or args[0] == "r":
                if self.is_RC:
                    self.transceiver.send(f"Adjusting rudder to {float(args[1])}")
                    self.rudder.angle = float(args[1])
                else:
                    self.transceiver.send("Refuse to change sail, not in RC Mode")

            elif args[0] == "sailoffset" or args[0] == "so":
                dataStr = String()
                dataStr.data = f"(driverOffset:sail:{float(args[1])})"
                self.get_logger().info(dataStr.data)
                self.pub.publish(dataStr)

            elif args[0] == "rudderoffset" or args[0] == "ro":
                dataStr = String()
                dataStr.data = f"(driverOffset:rudder:{float(args[1])})"
                self.get_logger().info(dataStr.data)
                self.pub.publish(dataStr)

            elif args[0] == "setevent":
                if args[1] not in events:
                    self.transceiver.send(f"Invalid event: {args[1]}")
                    return

                self.event = init_event(args[1], args[2:])
                self.is_RC = False

            elif args[0] == "goto":
                if not self.is_RC:
                    target = Waypoint(float(args[1]), float(args[2]))
                    self.transceiver.send(f"Going to: {target}")

            else:
                self.transceiver.send(f"Unknown command: {args[0]}")

        except IndexError:
            self.transceiver.send("Invalid length args for command")

        except Exception as e:
            self.transceiver.send(f"Error when parsing command: {e}")


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


if __name__ == "__main__":
    main()
