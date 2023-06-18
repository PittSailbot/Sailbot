import math
from datetime import date, datetime
from threading import Thread

import constants as c
from drivers import driver
from GPS import gps as Gps
from rclpy.node import Node
from transceiver import arduino
from windvane import windVane


class boat:
    def __init__(self):
        self._node = Node("findBuoyBoat")
        self.logging = self._node.get_logger()
        with open("boatMainLog.log", "a") as logfile:
            logfile.write("\n\n---------------------------------\n")

        self.gps = Gps()
        self.windvane = windVane()
        self.drivers = driver(sailAuto=False)
        self.arduino = arduino(c.config["MAIN"]["ardu_port"])

        self.currentTarget = None  # (longitude, latitude) tuple
        self.targets = []  # holds (longitude, latitude) tuples

        noGoZoneDegs = 20

        tempTarget = False

        pump_thread = Thread(target=self.pumpMessages)
        pump_thread.start()

    def move(self):
        if self.gps.distanceTo(currentTarget) < float(c.config["MAIN"]["acceptable_range"]) and len(self.targets) > 0:
            # next target

            targetAngle = TargetAngleRelativeToNorth()  # Func doesnt exist yet

            windAngleRelativeToNorth = convertWindAngle(self.windVane.angle)

            if tempTarget == False and targetAngle - windAngleRelativeToNorth < (noGoZoneDegs / 2):
                if targetAngle < windAngleRelativeToNorth or abs(targetAngle - 360) < windAngleRelativeToNorth:
                    # newTargetAngle < targetangle
                    newTargetAngle = windAngleRelativeToNorth - (noGoZoneDegs / 2 + 10)
                else:
                    # newTargetAngle > targetAngle
                    newTargetAngle = windAngleRelativeToNorth + (noGoZoneDegs / 2 + 10)

                tempTarget = True
                rotateToAngle(newTargetAngle)
                self.logging.info(f"Heading to temp target at: {newTargetAngle}")

            elif tempTarget and targetAngle - windAngleRelativeToNorth > (noGoZoneDegs / 2):
                tempTarget = False
                self.logging.info(f"Heading to target at: {targetAngle}")
                rotateToAngle(targetAngle)

            elif tempTarget == False:
                self.logging.info(f"Heading to target at: {targetAngle}")
                rotateToAngle(targetAngle)

            # else:
            #     self.currentTarget = self.targets.pop(0)

        else:
            # move towards target
            self.adjustSail()
            self.adjustRudder()

    def adjustSail(self):
        if self.currentTarget:
            windDir = self.windvane.angle
            targetAngle = windDir + 35
            self.drivers.sail.set(targetAngle)
            self.logging.info(f"Adjusted sail to: {targetAngle}")
        else:
            # move sail to home position
            self.drivers.sail.set(0)
            self.logging.info("Adjusted sail to home position")

    def adjustRudder(self):
        if self.currentTarget:
            # adjust rudder for best wind
            angleTo = gps.angleTo(self.currentTarget)
            d_angle = angleTo - gps.track_angle_deg

            if d_angle > 180:
                d_angle -= 180

            self.drivers.rudder.set(d_angle)
            self.logging.info(f"Adjusted rudder to: {d_angle}")

        else:
            # move sail to home position
            self.drivers.rudder.set(0)
            self.logging.info("Adjusted rudder to home position")

    def pumpMessages(self):
        while True:
            self.readMessages()

    def readMessages(self):
        msgs = self.arduino.read()
        print(msgs)

        # for msg in msgs:
        if True:
            msg = msgs
            ary = msg.split(" ")
            if len(ary) > 1:
                if ary[0] == "sail":
                    self.drivers.sail.set(float(ary[1]))
                    self.logging.info("Received message to adjust sail")
                elif ary[0] == "rudder":
                    self.drivers.rudder.set(float(ary[1]))
                    self.logging.info("Received message to adjust rudder")
                elif ary[0] == "mode":
                    print("TODO: add Modes")

    # //////

    def find_buoy(self, buoylat, buoylong):
        radiusConst = 100  # meter
        radius = radiusConst / 111139  # degree of lat/long

        # find angle of boat from center of radius
        gpslat = self.gps.latitude
        gpslong = self.gps.longitude

        a = gpslat - buoylat
        b = gpslong - buoylong
        ang = math.atan(b / a)
        ang *= 180 / math.pi

        if a < 0:
            ang += 180

        # finding points along radius boat should move to
        # after initial entry (first point)

        # list of next targets
        # 72 = 360/num-of-points(here its 5)
        tar_angs = [ang, ang + 72, ang - 72, ang - (72 * 3), ang - (72 * 2)]
        tarx = [0] * 5
        tary = [0] * 5

        for i in range(0, 5):
            tarx[i] = buoylat + radius * math.cos(tar_angs[i] * (math.pi / 180))
            tary[i] = buoylong + radius * math.sin(tar_angs[i] * (math.pi / 180))

        for i in range(0, 5):
            # spotbuoy is fake commands idk what the real ones are
            self.goToGPS(tarx[i], tary[i])

            if spotbuoy == True:
                break


if __name__ == "__main__":
    boat()
