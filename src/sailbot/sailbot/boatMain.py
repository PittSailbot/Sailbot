import importlib
import os
import sys
import traceback
from threading import Thread
from time import sleep

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import src.sailbot.sailbot.utils.boatMath as boatMath
import constants as c
import src.sailbot.sailbot.utils as utils
from src.sailbot.sailbot.utils.eventUtils import EventFinished, Waypoint, getDefaultEventParams, getEventDict

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if DOCKER == "True" else False

# import the peripherals from the appropriate folder
folder = "sailbot.peripherals" if not DOCKER else "sailbot.virtualPeripherals."
windVane = importlib.import_module(folder + "windvane").windVane
gps = importlib.import_module(folder + "GPS").gps
compass = importlib.import_module(folder + "compass").compass
drivers = importlib.import_module(folder + "drivers").driver
arduino = importlib.import_module(folder + "transceiver").arduino


class boat(Node):

    """
    The overarching class for the entire boat, contains all sensor objects and automation functions
    """

    def __init__(self, calibrateOdrive=True):
        """
        Set everything up and start the main loop
        """
        super().__init__("main")
        self.logging = self.get_logger()

        # create sensor objects
        self.createSensorWrappers()
        self.windvane = windVane()

        self.gps_subscription = self.create_subscription(
            String, "GPS", self.ROS_GPSCallback, 10
        )
        self.compass_subscription = self.create_subscription(
            String, "compass", self.ROS_compassCallback, 10
        )
        self.mode_subscription = self.create_subscription(
            String, "mode", self.ROS_modeCallback, 10
        )

        self.driverPub = self.create_publisher(String, "driver", 10)
        self.modePub = self.create_publisher(String, "mode", 10)

        # try both of the USB ports the 'arduino' (transciver) may be connected to
        try:
            self.arduino = arduino(c.config["MAIN"]["ardu_port"])
            if self.arduino.readData() == "'":
                raise Exception("Could not read arduino")
            else:
                self.logging.debug(self.arduino.readData())
        except:
            self.arduino = arduino(c.config["MAIN"]["ardu_port2"])
            if self.arduino.readData() == "'":
                raise Exception("Could not read arduino")

        # Set default values for variables
        self.currentEvent = None
        self.event_params = {"gps": self.gps}
        self.cycleTargets = False
        self.currentTarget = None  # (longitude, latitude) tuple
        self.targets = []  # list of (longitude, latitude) tuples

        self.currentSail = 0
        self.currentRudder = 0

        self.modeSetting = c.config["MODES"]["MOD_RC"]
        self.manualControl = True  # check RC Mode to change manualControl, and manualControl checks for everything else (faster on memory)

        # generate the event dictionary (takes some time)
        Thread(target=getEventDict).start()

        self.logging.info("boatMain initialized")

    def ROS_GPSCallback(self, string):
        string = string.data
        if string == "None,None,None":
            self.gps.latitude = None
            self.gps.longitude = None
            self.gps.waypoint = None
            self.gps.track_angle_deg = None
            return

        lat, lon, trackangle = string.replace("(", "").replace(")", "").split(",")
        self.gps.latitude = float(lat)
        self.gps.longitude = float(lon)
        self.gps.track_angle_deg = float(trackangle)
        self.gps.waypoint = Waypoint(float(lat), float(lon))

    def ROS_compassCallback(self, string):
        string = string.data
        if string == "None,None":
            self.compass.angle = None
            return

        angle = string.replace("(", "").replace(")", "")
        self.compass.angle = float(angle)

    def ROS_modeCallback(self, string):
        string = string.data
        if string.split(":")[0] == "Set Mode":
            newMode = string.split(":")[1]
            found = False
            eventDict = (
                getEventDict()
            )  # this imports the events and therefore can take a while the first time it's called
            eventParams = {**getDefaultEventParams(newMode), **self.event_params}
            for key in eventDict.keys():
                if newMode.lower() == key.lower():
                    try:
                        self.currentEvent = eventDict[key](eventParams)
                        self.modeSetting = key
                        self.logging.info(f"Set mode to {key}")
                    except Exception as e:
                        self.modeSetting = c.config["MODES"]["MOD_RC"]
                        self.logging.error(
                            f"Failed to set mode to {key} with error: {e}\n"
                            + traceback.format_exc()
                        )
                    found = True
                    break

            if found == False:
                self.logging.warning(f"Mode Not Found: {newMode}")

            elif (
                self.modeSetting == c.config["MODES"]["MOD_RC"]
                and self.manualControl is False
            ):
                self.logging.info("Setting manual mode to True")
                self.manualControl = True

            elif (
                self.modeSetting != c.config["MODES"]["MOD_RC"]
                and self.manualControl is True
            ):
                self.logging.info("Setting manual mode to False")
                self.manualControl = False

        else:
            self.logging.warning(f"Unknown command in ROS_modeCallback: {string}")

    def createSensorWrappers(self):
        self.gps = utils.dummyObject()
        self.gps.latitude = 0.0
        self.gps.longitude = 0.0
        self.gps.track_angle_deg = 0.0
        self.gps.waypoint = Waypoint(0.0, 0.0)

        self.compass = utils.dummyObject()  # compass()
        self.compass.angle = 0.0

    def adjustSail(self, angle=None):
        """
        Move the sail to 'angle', angle is value between 0 and 90, 0 being all the way in
        defaults to optimum when angle is None
        """
        dataStr = String()
        if self.manualControl and angle != None:
            # set sail to angle
            dataStr.data = f"(driver:sail:{angle})"
            self.get_logger().debug(dataStr.data)
            self.driverPub.publish(dataStr)
            self.currentSail = angle

        elif self.currentTarget or self.manualControl:
            # set sail to optimal angle based on windvane readings
            windDir = self.windvane.angle
            targetAngle = windDir + 35
            dataStr.data = f"(driver:sail:{targetAngle})"
            self.get_logger().debug(dataStr.data)
            self.driverPub.publish(dataStr)
            self.currentSail = angle

        else:
            # move sail to home position
            dataStr.data = f"(driver:sail:{0})"
            self.driverPub.publish(dataStr)
            self.currentSail = 0
            self.logging.debug("Adjusted sail to home position")

    def adjustRudder(self, angle):
        """
        Move the rudder to 'angle', angle is value between -45 and 45
        """
        dataStr = String()
        if self.currentTarget or self.manualControl == True:
            dataStr.data = f"(driver:rudder:{angle})"
            self.driverPub.publish(dataStr)
            self.currentRudder = angle
            self.logging.debug(
                f"Adjusted rudder to: {angle}",
            )

        else:
            # move rudder to home position
            dataStr.data = f"(driver:rudder:{0})"
            self.driverPub.publish(dataStr)
            self.currentRudder = 0
            self.logging.debug("Adjusted rudder to home position")

    def mainLoop(self):
        """
        Main loop that will run until boat is stopped
        """
        while True:
            self.readMessages()  # read messages from transceiver

            if not self.manualControl and self.currentEvent:
                try:
                    targetWaypoint = self.currentEvent.next_gps()
                    self.currentTarget = targetWaypoint
                    self.goToGPS(targetWaypoint)

                except EventFinished:
                    # end of event
                    self.logging.info("Switching to RC")
                    self.currentEvent = None
                    modeStr = String()
                    modeStr.data = c.config["MODES"]["MOD_RC"]
                    self.modePub.publish(modeStr)

                except Exception as e:
                    self.logging.error(f"Event error: {e}\n" + traceback.format_exc())
                    self.currentEvent = None
                    modeStr = String()
                    modeStr.data = c.config["MODES"]["MOD_RC"]
                    self.modePub.publish(modeStr)

    def sendData(self):
        # GPS:x/y, RudderPos, SailPos, BoatOrientation, Windspd, WindDir, Batt
        # 1/2,3,4,5,6,7,8
        totstr = ""
        arr = ["N/a"] * 8

        try:
            arr[0] = f"{self.gps.longitude}"
            arr[1] = f"{self.gps.latitude}"
        except Exception as e:
            self.logging.warning(f"failed to find data: gps, {e}")
            self.logging.warning(f"data error: {e}")

        try:
            if abs(self.currentRudder) >= 10:
                arr[2] = f"{self.currentRudder}"
            else:  # format so number is 2 digits
                arr[2] = f"0{self.currentRudder}"
        except Exception as e:
            self.logging.warning(f"failed to find data: rudder, {e}")
            self.logging.warning(f"data error: {e}")

        try:
            if abs(self.currentSail) >= 10:
                arr[3] = f"{self.currentSail}"
            else:  # format so number is 2 digits
                arr[3] = f"0{self.currentSail}"
        except Exception as e:
            self.logging.warning(f"failed to find data: sail, {e}")
            self.logging.warning(f"data error: {e}")

        try:
            arr[4] = f"{self.compass.angle}"
        except Exception as e:
            self.logging.warning(f"failed to find data: compass, {e}")
            self.logging.warning(f"data error: {e}")

        try:
            # arr[5] = #dont have
            arr[6] = f"{self.windvane.angle}"
        except Exception as e:
            self.logging.warning(f"failed to find data: windvane, {e}")
            self.logging.warning(f"data error: {e}")

        # arr[7] = #dont have

        for i in range(8):
            totstr += arr[i]
            if i < 7:
                totstr += ","

        self.arduino.send("DATA: " + totstr)

    def readMessages(self):
        """
        Read messages from transceiver
        """
        msgs = self.arduino.readData()

        for msg in msgs:
            try:
                ary = msg.split(" ")
                # make it so you cant do manual commands unless in RC mode to avoid automation undoing work done
                # override is in place to switch to RC if given RC commands if potential accidents may happen
                if len(ary) > 0:
                    # manual adjust sail
                    if ary[0] == "sail" or ary[0] == "S":
                        if self.manualControl:
                            self.logging.debug(
                                f"Received message to adjust sail to {float(ary[1])}"
                            )
                            self.adjustSail(float(ary[1]))

                    # manual adjust rudder
                    elif ary[0] == "rudder" or ary[0] == "R":
                        if self.manualControl:
                            rudderMidPoint = (
                                float(c.config["CONSTANTS"]["rudder_angle_max"])
                                - float(c.config["CONSTANTS"]["rudder_angle_min"])
                            ) / 2
                            halfDeadZone = (
                                float(
                                    c.config["CONSTANTS"][
                                        "ControllerRudderDeadZoneDegs"
                                    ]
                                )
                                / 2
                            )

                            # ignore values within the dead zone, ex: if controller is at 1 degree, rudder will still default to 0
                            if (
                                float(ary[1]) < rudderMidPoint + halfDeadZone
                                and float(ary[1]) > rudderMidPoint - halfDeadZone
                            ):
                                targetRudder = (
                                    float(rudderMidPoint) - 45
                                )  # values read in range from 0:90 instead of -45:45
                            else:
                                targetRudder = (
                                    float(ary[1]) - 45
                                )  # values read in range from 0:90 instead of -45:45
                            self.adjustRudder(targetRudder)
                            self.logging.debug(
                                f"Received message to adjust rudder to {targetRudder}"
                            )

                    elif str(ary[0]) == "sailOffset" or ary[0] == "SO":
                        self.manualControl = True
                        dataStr = String()
                        dataStr.data = f"(driverOffset:sail:{float(ary[1])})"
                        self.logging.debug(dataStr.data)
                        self.driverPub.publish(dataStr)

                    elif str(ary[0]) == "rudderOffset" or ary[0] == "RO":
                        self.manualControl = True
                        dataStr = String()
                        dataStr.data = f"(driverOffset:rudder:{float(ary[1])})"
                        self.logging.debug(dataStr.data)
                        self.driverPub.publish(dataStr)

                    elif (
                        ary[0] == "mode"
                    ):  # set boats mode to the value specified, command should be formatted : "mode {value}" value is int 1-5
                        try:
                            if int(ary[1]) < 0 or int(ary[1]) > 5:
                                self.logging.warning("Outside mode range")
                            else:
                                self.logging.info(f"Setting mode to {int(ary[1])}")
                                modeStr = String()
                                modeStr.data = int(ary[1])
                                self.modePub.publish(modeStr)

                        except Exception as e:
                            self.logging.error(f"Error changing mode: {e}")

                    elif ary[0] == "addTarget":  # add current GPS to list of targets
                        while self.gps.latitude == None or self.gps.longitude == None:
                            self.logging.warning("no gps")
                            sleep(0.1)

                        target = (self.gps.latitude, self.gps.longitude)
                        self.targets.append(target)
                        self.logging.info(
                            f"added target, current target list is {self.targets}"
                        )

                    elif ary[0] != "":
                        self.logging.warning(f"unknown command {ary[0]}")

            except Exception as e:
                self.logging.warning(f"failed to run command {msg}, gave error: {e}")

    def goToGPS(self, waypoint):
        """
        Go to GPS coordinates defined by waypoint
        """
        lat = waypoint.lat
        lon = waypoint.lon
        # Get current GPS coordinates, if we can't load info from GPS wait until we can and print error message

        while self.gps.latitude == None or self.gps.longitude == None:
            self.logging.warning("no gps")
            sleep(0.1)

        # determine angle we need to turn
        compassAngle = self.compass.angle
        deltaAngle = boatMath.angleToPoint(
            compassAngle, self.gps.latitude, self.gps.longitude, lat, lon
        )
        targetAngle = (compassAngle + deltaAngle) % 360
        windAngle = self.windvane.angle

        if (deltaAngle + windAngle) % 360 < self.windvane.noGoMin and (
            deltaAngle + windAngle
        ) % 360 > self.windvane.noGoMax:  # angle is not in no go zone
            self.turnToAngle(targetAngle)
        else:  # angle is in no go zone, go to the nearest edge of no go zone
            if (targetAngle - compassAngle) % 360 <= 180:
                # turn left
                self.turnToAngle(self.windvane.noGoMin)
            else:
                # turn right
                self.turnToAngle(self.windvane.noGoMax)

        if abs(
            boatMath.distanceInMBetweenEarthCoordinates(
                lat, lon, self.gps.latitude, self.gps.longitude
            )
        ) < int(c.config["CONSTANTS"]["reachedGPSThreshhold"]):
            # if we are very close to GPS coord
            if self.cycleTargets:
                self.targets.append((lat, lon))
            self.currentTarget = None
            self.adjustRudder(0)

    def turnToAngle(self, angle):
        Thread(target=self.turnToAngle_blocking, args=[angle]).start()

    def turnToAngle_blocking(self, angle):
        """
        adjust rudder until the boat is facing compass angle 'angle'
        """
        leftPositive = -1  # change to negative one if boat is rotating the wrong way

        self.logging.debug(f"starting turnToAngle: {angle}")
        compassAngle = self.compass.angle
        while abs(compassAngle - angle) > int(
            c.config["CONSTANTS"]["angle_margin_of_error"]
        ):  # while not facing the right angle
            compassAngle = self.compass.angle

            if (angle - compassAngle) % 360 <= 180:  # turn Left
                # move rudder to at most 45 degrees
                rudderPos = leftPositive * min(
                    45, 3 * abs(compassAngle - angle)
                )  # /c.rotationSmoothingConst)
                self.logging.debug(
                    f"turning to angle: {angle} from angle: {compassAngle} by turning rudder to {rudderPos}"
                )
                self.adjustRudder(int(rudderPos))
            else:
                # move rudder to at most -45 degrees
                rudderPos = (
                    -1 * leftPositive * min(45, 3 * abs(compassAngle - angle))
                )  # /c.rotationSmoothingConst)
                self.logging.debug(
                    f"turning to angle: {angle} from angle: {compassAngle} by turning rudder to {rudderPos}"
                )
                self.adjustRudder(int(rudderPos))

        self.logging.info(f"finished turnToAngle {angle}")


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/main"
    rclpy.init(args=args)

    calibrateOdrive = True
    for arg in sys.argv:
        if arg == "noCal":
            calibrateOdrive = False
    b = boat(calibrateOdrive=calibrateOdrive)
    try:
        Thread(target=b.mainLoop).start()
        rclpy.spin(b)
        b.logging.info("Cleaning Up")
        b.adjustRudder(0)
        b.adjustSail(0)

        b.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt as e:
        b.logging.info("\n\nEXITING...\n\n")
        b.adjustRudder(0)
        b.adjustSail(0)
        b.destroy_node()
        rclpy.shutdown()
        b.logging.info("EXITED CLEANLY")


if __name__ == "__main__":
    """
    this is the code that runs if you run this file
    read in command line args and create boat object
    start mainloop
    when code is stopped return sail and rudder to 0 position
    """
    main()
