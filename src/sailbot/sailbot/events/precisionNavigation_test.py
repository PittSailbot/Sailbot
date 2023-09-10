import importlib
import os
import sys

import sailbot.boatMath as boatMath
import sailbot.constants as c
# from camera import camera
from sailbot.events.endurance import Endurance
# import Precision_Navigation,Endurance,Station_Keeping,Search
from sailbot.events.eventUtils import Event, EventFinished, Waypoint
from sailbot.events.stationKeeping import Station_Keeping

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if DOCKER == "True" else False
folder = "sailbot.peripherals" if not DOCKER else "sailbot.virtualPeripherals."

camera = importlib.import_module(folder + "camera").Camera
gps = importlib.import_module(folder + "GPS").gps
arduino = importlib.import_module(folder + "transceiver").arduino
driver = importlib.import_module(folder + "drivers").driver
windVane = importlib.import_module(folder + "windvane").windVane


import math

if __name__ == "__main__":
    sys.exit(1)

from datetime import date, datetime
from threading import Thread
from time import sleep, time

import numpy
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

"""
# Challenge	Goal:
    - To demonstrate the boat's ability to autonomously navigate a course within tight tolerances.
        
    # Description:
        - The boat will start between two buoys
        - then will autonomously sail a course around two buoys
        - then return between the two start buoys
        
    # Scoring:
        - 10 pts max
        - 2 pts/each for rounding the first two buoys
        - 6 pts more for finishing between the start buoys
        - or 4 pts more for crossing the line outside of the start buoys.
        
    # Assumptions: (based on guidelines)
        - behind start is upstream
        - going back is harder
    
    # Strategy:
        - TODO write psuedocode about how this event logic works
"""

REQUIRED_ARGS = 4


class Precision_Navigation:
    """
    Attributes:
        - event_info (array) - location of buoys to sail around
            event_info = [(start_lat, start_long), (b1_lat, b1_long), (b2_lat, b2_long)]
    """

    def __init__(self, event_info_inp):
        self._node = Node("precisionNavEventTest")
        self.logging = self._node.get_logger()
        if len(event_info_inp) != REQUIRED_ARGS:
            raise TypeError(
                f"Expected {REQUIRED_ARGS} arguments, got {len(event_info_inp)}"
            )

        self.logging.info("Percision Navigation moment")
        self.event_info = event_info_inp

        self.ifsideways = None
        self.ifupsidedown = None  # set up for later PN_checkwayside

        self.PN_arr = []
        self.PN_arr = self.PN_coords()

        self.rev_bool = False
        self.target_set = 1
        self.start_time = time.time()
        self.manualControl = False

    def adjustRudder(self, angleTo):
        """
        Move the rudder to 'angle', angle is value between -45 and 45
        """
        dataStr = String()
        if self.currentTarget or self.manualControl == True:
            # adjust rudder for best wind
            # angleTo = gps.angleTo(self.currentTarget)

            d_angle = angleTo - gps.track_angle_deg

            if d_angle > 180:
                d_angle -= 180

            dataStr.data = f"(driver:rudder:{d_angle})"
            self.pub.publish(dataStr)

            self.currentRudder = d_angle
            self.logging.debug(f"Adjusted rudder to: {d_angle}")

        else:
            # move rudder to home position
            dataStr.data = f"(driver:rudder:{0})"
            self.pub.publish(dataStr)

            self.currentRudder = 0
            self.logging.debug("Adjusted rudder to home position")

    def adjustSail(self, angle=None):
        """
        Move the sail to 'angle', angle is value between 0 and 90, 0 being all the way in
        """
        dataStr = String()
        if self.manualControl and angle != None:
            # set sail to angle
            dataStr.data = f"(driver:sail:{angle})"
            self.logging.debug(dataStr.data)
            self.pub.publish(dataStr)

        elif self.currentTarget or self.manualControl:
            # set sail to optimal angle based on windvane readings
            windDir = windVane.angle
            targetAngle = windDir + 35
            dataStr.data = f"(driver:sail:{targetAngle})"
            self.loggin.debug(dataStr.data)
            self.pub.publish(dataStr)
            self.currentSail = targetAngle

        else:
            # move sail to home position
            dataStr.data = f"(driver:sail:{0})"
            self.pub.publish(dataStr)
            self.currentSail = 0
            self.logging.debug("Adjusted sail to home position")

    def goToGPS(self, lat, long):
        """
        Go to GPS coordinates lat, long
        """

        # Get current GPS coordinates, if we can't load info from GPS wait until we can and print error message
        # gps.updategps()
        while gps.latitude == None or gps.longitude == None:
            self.logging.warning("no gps")
            # gps.updategps()
            sleep(0.1)

        # determine angle we need to turn
        compassAngle = compass.angle
        deltaAngle = boatMath.angleToPoint(
            compassAngle, gps.latitude, gps.longitude, lat, long
        )
        targetAngle = (compassAngle + deltaAngle) % 360
        windAngle = windVane.angle

        if (deltaAngle + windAngle) % 360 < windVane.noGoMin and (
            deltaAngle + windAngle
        ) % 360 > windVane.noGoMax:  # angle is not in no go zone
            self.turnToAngle(targetAngle)
        else:  # angle is in no go zone, go to the nearest edge of no go zone
            if (targetAngle - compassAngle) % 360 <= 180:
                # turn left
                self.turnToAngle(windVane.noGoMin)
            else:
                # turn right
                self.turnToAngle(windVane.noGoMax)

        if abs(
            boatMath.distanceInMBetweenEarthCoordinates(
                lat, long, gps.latitude, gps.longitude
            )
        ) < int(c.config["CONSTANTS"]["reachedGPSThreshhold"]):
            # if we are very close to GPS coord
            if self.cycleTargets:
                self.targets.append((lat, long))
            self.currentTarget = None
            self.adjustRudder(0)

    def turnToAngle(self, angle, wait_until_finished=False):
        """
        adjust rudder until the boat is facing compass angle 'angle'
        if wait_until_finished is True the boat will continue to adjust rudder until the boat is facing the right direction
        if wait_until_finished is False the boat will adjust the rudder to an appropriate angle and then return
        """
        leftPositive = -1  # change to negative one if boat is rotating the wrong way

        self.logging.debug("starting turnToAngle")
        compassAngle = compass.angle
        while abs(compassAngle - angle) > int(
            c.config["CONSTANTS"]["angle_margin_of_error"]
        ):  # while not facing the right angle
            compassAngle = compass.angle

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

            if not wait_until_finished:
                break

        self.logging.debug("finished turnToAngle")

    #########################################

    def next_gps(self):
        """
        Main event script logic. Executed continuously by boatMain.

        Returns either:
            - The next GPS point that the boat should sail to stored as a Waypoint object
            - OR None to signal the boat to drop sails and clear waypoint queue
            - OR EventFinished exception to signal that the event has been completed
        """
        # time based check
        # curr_time = time.time()
        # if int(curr_time - self.start_time)%4 != 0: return None,None

        # set check
        if self.target_set >= int(len(self.PN_arr) / 2):
            self.logging.info(f"PN: REACHED FINAL POINT;\nEXITING EVENT")
            raise EventFinished

            # main running: go to points in PN_arr
        if self.PN_PassCheck():
            self.logging.info(f"PN: PASSED TARGET POINT: {self.target_set}")
            self.target_set += 1

        self.logging.info(f"PN: CURR TARGET POINT: {self.target_set}")
        return Waypoint(
            self.PN_arr[self.target_set * 2], self.PN_arr[(self.target_set * 2) + 1]
        )

    # find coords that should go to via cart of buoy coords
    def PN_coords(self):
        # adjustable values
        rad1 = (4 / 6371000) * (180 / math.pi)  # inner rad
        rad2 = (8 / 6371000) * (180 / math.pi)  # outer rad
        m1 = 45
        m2 = -15  # rad offset from 90 and 225/-45 points
        # see desmos: https://www.desmos.com/calculator/2fjqthukuf

        ret_arr = []
        # pt1= b3 rad1: 90+m1
        # pt2= b3 rad2: 225+m2
        # pt3= between b2 and b4
        # pt4= b4 rad2:315+m2
        # pt5= b4 rad1:90+m1
        # pt6= between b1 and b2

        # calcing x/y points
        t = math.pi / 180  # conv deg to rad
        # b3[0-3]
        ret_arr.append(rad1 * math.cos((90 + m1) * t) + self.event_info[4])  # pt1x[0]
        ret_arr.append(rad1 * math.sin((90 + m1) * t) + self.event_info[5])  # pt1y[1]
        ret_arr.append(rad2 * math.cos((225 + m2) * t) + self.event_info[4])  # pt2x[2]
        ret_arr.append(rad2 * math.sin((225 + m2) * t) + self.event_info[5])  # pt2y[3]

        # b3/4[4-5]
        ret_arr.append((self.event_info[4] + self.event_info[6]) / 2)  # pt3x[4]
        ret_arr.append((self.event_info[5] + self.event_info[7]) / 2)  # pt3y[5]

        # b4[6-9]
        ret_arr.append(rad2 * math.cos((315 - m2) * t) + self.event_info[6])  # pt4x[6]
        ret_arr.append(rad2 * math.sin((315 - m2) * t) + self.event_info[7])  # pt4y[7]
        ret_arr.append(rad1 * math.cos((90 - m1) * t) + self.event_info[6])  # pt5x[8]
        ret_arr.append(rad1 * math.sin((90 - m1) * t) + self.event_info[7])  # pt5y[9]

        # b1/2[10-11]
        ret_arr.append((self.event_info[0] + self.event_info[2]) / 2)  # pt6x[10]
        ret_arr.append((self.event_info[1] + self.event_info[3]) / 2)  # pt6y[11]

        return ret_arr

    # find if passed target (ret bool)
    def PN_PassCheck(self):
        # self.target_set,self.PN_arr
        # self.event_info

        """
        #SK_f(x)
        #P1[0-1], self.event_info[4-5]
        #P1[2-3], self.event_info[4-5]
        #
        #P1[6-7], self.event_info[6-7]
        #P1[8-9], self.event_info[6-7]
        #self.event_info[0-1], self.event_info[2-3]

        #pt1: x<P1x[0],    y<L1(x)[ m(P1[0-1],[]) ]
        #pt2: x>P2x[2],    y<L2(x)
        #pt3: x>Perpendicular P2toP4    -(1/m)x+ P[5]-(1/m)*P[6]
        #pt4: x>P4x[6],    y>L4(x)
        #pt5: x<P5x[8],    y>L5(x)
        #pt6: y>L6(x)
        """

        gps.updategps()
        # gps.longitude
        # gps.latitude

        # TODO:
        # either figure out better system, or put in a 'reverse if' of the next case in each statement to know whether the coords are right
        # Ie: if x<whatever before it starts the next set, check for x>=whatever in the next case
        # mult by a 'rev_bool' decided
        # that really sucks to map out though

        if self.ifupsidedown == None:
            (
                self.ifupsidedown,
                self.ifsideways,
            ) = self.PN_checkwayside()  # True=sideways/upsidedown
        # [x_]upsidedown: flip >/<
        # [_x]sideways: flip long/lat
        # 00,01,10,11: standard;turn 90deg left for order
        x = False

        # standard------------------------------------------------------------------
        if not (self.ifupsidedown) and not (self.ifsideways):
            # left(long) of BL buoy[{4},5]
            # below(lat) line between p1[0,1] and BL buoy[4,5]
            if self.target_set == 1:
                x = gps.longitude <= self.event_info[4] and gps.latitude <= self.SK_f(
                    gps.longitude,
                    self.PN_arr[0],
                    self.PN_arr[1],
                    self.event_info[4],
                    self.event_info[5],
                )

            # below(lat) of BL buoy[4,{5}]
            # below(lat) line between p2[2,3] and BL buoy[4,5]
            elif self.target_set == 2:
                x = gps.latitude <= self.event_info[5] and gps.latitude <= self.SK_f(
                    gps.longitude,
                    self.PN_arr[2],
                    self.PN_arr[3],
                    self.event_info[4],
                    self.event_info[5],
                )

            # right(long) of line perpendicular to p2[2,3] and p4[6,7] at p3[4,5]
            elif self.target_set == 3:
                x = gps.longitude >= self.PN_Perpend(
                    gps.longitude,
                    self.PN_arr[4],
                    self.PN_arr[5],
                    self.PN_arr[2],
                    self.PN_arr[3],
                    self.PN_arr[4],
                    self.PN_arr[5],
                )

            # right(long) of BR buoy[{6},7]
            # above(lat) line between p4[6,7] and BR buoy[6,7]
            elif self.target_set == 4:
                x = gps.longitude >= self.event_info[6] and gps.latitude >= self.SK_f(
                    gps.longitude,
                    self.PN_arr[6],
                    self.PN_arr[7],
                    self.event_info[6],
                    self.event_info[7],
                )

            # above(lat) of BR buoy[6,{7}]
            # above(lat) line between p5[8,9] and BR buoy[6,7]
            elif self.target_set == 5:
                x = gps.latitude >= self.event_info[7] and gps.latitude >= self.SK_f(
                    gps.longitude,
                    self.PN_arr[8],
                    self.PN_arr[9],
                    self.event_info[6],
                    self.event_info[7],
                )

            # above(lat) line between TL[0,1] and TR[2,3]
            elif self.target_set == 6:
                x = gps.latitude >= self.SK_f(
                    gps.longitude,
                    self.event_info[0],
                    self.event_info[1],
                    self.event_info[2],
                    self.event_info[3],
                )
            else:
                self.logging.info(
                    f"PN: ERROR: 00: TARGET SET OUT OF RANGE (1to6)\nTARGET PNT = {self.target_set}"
                )

        # flip long/lat------------------------------------------------------------------
        elif not (self.ifupsidedown) and self.ifsideways:
            # left(long) of BL buoy[{4},5]
            # below(lat) line between p1[0,1] and BL buoy[4,5]
            if self.target_set == 1:
                x = gps.latitude <= self.event_info[4] and gps.longitude <= self.SK_f(
                    gps.latitude,
                    self.PN_arr[0],
                    self.PN_arr[1],
                    self.event_info[4],
                    self.event_info[5],
                )

            # below(lat) of BL buoy[4,{5}]
            # below(lat) line between p2[2,3] and BL buoy[4,5]
            elif self.target_set == 2:
                x = gps.longitude <= self.event_info[5] and gps.longitude <= self.SK_f(
                    gps.latitude,
                    self.PN_arr[2],
                    self.PN_arr[3],
                    self.event_info[4],
                    self.event_info[5],
                )

            # right(long) of line perpendicular to p2[2,3] and p4[6,7] at p3[4,5]
            elif self.target_set == 3:
                x = gps.latitude >= self.PN_Perpend(
                    gps.latitude,
                    self.PN_arr[4],
                    self.PN_arr[5],
                    self.PN_arr[2],
                    self.PN_arr[3],
                    self.PN_arr[4],
                    self.PN_arr[5],
                )

            # right(long) of BR buoy[{6},7]
            # above(lat) line between p4[6,7] and BR buoy[6,7]
            elif self.target_set == 4:
                x = gps.latitude >= self.event_info[6] and gps.longitude >= self.SK_f(
                    gps.latitude,
                    self.PN_arr[6],
                    self.PN_arr[7],
                    self.event_info[6],
                    self.event_info[7],
                )

            # above(lat) of BR buoy[6,{7}]
            # above(lat) line between p5[8,9] and BR buoy[6,7]
            elif self.target_set == 5:
                x = gps.longitude >= self.event_info[7] and gps.longitude >= self.SK_f(
                    gps.latitude,
                    self.PN_arr[8],
                    self.PN_arr[9],
                    self.event_info[6],
                    self.event_info[7],
                )

            # above(lat) line between TL[0,1] and TR[2,3]
            elif self.target_set == 6:
                x = gps.longitude >= self.SK_f(
                    gps.latitude,
                    self.event_info[0],
                    self.event_info[1],
                    self.event_info[2],
                    self.event_info[3],
                )
            else:
                self.logging.info(
                    f"PN: ERROR: 00: TARGET SET OUT OF RANGE (1to6)\nTARGET PNT = {self.target_set}"
                )

        # flip >/<------------------------------------------------------------------
        elif self.ifupsidedown and not (self.ifsideways):
            # left(long) of BL buoy[{4},5]
            # below(lat) line between p1[0,1] and BL buoy[4,5]
            if self.target_set == 1:
                x = gps.longitude >= self.event_info[4] and gps.latitude >= self.SK_f(
                    gps.longitude,
                    self.PN_arr[0],
                    self.PN_arr[1],
                    self.event_info[4],
                    self.event_info[5],
                )

            # below(lat) of BL buoy[4,{5}]
            # below(lat) line between p2[2,3] and BL buoy[4,5]
            elif self.target_set == 2:
                x = gps.latitude >= self.event_info[5] and gps.latitude >= self.SK_f(
                    gps.longitude,
                    self.PN_arr[2],
                    self.PN_arr[3],
                    self.event_info[4],
                    self.event_info[5],
                )

            # right(long) of line perpendicular to p2[2,3] and p4[6,7] at p3[4,5]
            elif self.target_set == 3:
                x = gps.longitude <= self.PN_Perpend(
                    gps.longitude,
                    self.PN_arr[4],
                    self.PN_arr[5],
                    self.PN_arr[2],
                    self.PN_arr[3],
                    self.PN_arr[4],
                    self.PN_arr[5],
                )

            # right(long) of BR buoy[{6},7]
            # above(lat) line between p4[6,7] and BR buoy[6,7]
            elif self.target_set == 4:
                x = gps.longitude <= self.event_info[6] and gps.latitude <= self.SK_f(
                    gps.longitude,
                    self.PN_arr[6],
                    self.PN_arr[7],
                    self.event_info[6],
                    self.event_info[7],
                )

            # above(lat) of BR buoy[6,{7}]
            # above(lat) line between p5[8,9] and BR buoy[6,7]
            elif self.target_set == 5:
                x = gps.latitude <= self.event_info[7] and gps.latitude <= self.SK_f(
                    gps.longitude,
                    self.PN_arr[8],
                    self.PN_arr[9],
                    self.event_info[6],
                    self.event_info[7],
                )

            # above(lat) line between TL[0,1] and TR[2,3]
            elif self.target_set == 6:
                x = gps.latitude <= self.SK_f(
                    gps.longitude,
                    self.event_info[0],
                    self.event_info[1],
                    self.event_info[2],
                    self.event_info[3],
                )
            else:
                self.logging.info(
                    f"PN: ERROR: 00: TARGET SET OUT OF RANGE (1to6)\nTARGET PNT = {self.target_set}"
                )

        # flip long/lat, flip >/<------------------------------------------------------------------
        elif self.ifupsidedown and self.ifsideways:
            # left(long) of BL buoy[{4},5]
            # below(lat) line between p1[0,1] and BL buoy[4,5]
            if self.target_set == 1:
                x = gps.latitude >= self.event_info[4] and gps.longitude >= self.SK_f(
                    gps.latitude,
                    self.PN_arr[0],
                    self.PN_arr[1],
                    self.event_info[4],
                    self.event_info[5],
                )

            # below(lat) of BL buoy[4,{5}]
            # below(lat) line between p2[2,3] and BL buoy[4,5]
            elif self.target_set == 2:
                x = gps.longitude >= self.event_info[5] and gps.longitude >= self.SK_f(
                    gps.latitude,
                    self.PN_arr[2],
                    self.PN_arr[3],
                    self.event_info[4],
                    self.event_info[5],
                )

            # right(long) of line perpendicular to p2[2,3] and p4[6,7] at p3[4,5]
            elif self.target_set == 3:
                x = gps.latitude <= self.PN_Perpend(
                    gps.latitude,
                    self.PN_arr[4],
                    self.PN_arr[5],
                    self.PN_arr[2],
                    self.PN_arr[3],
                    self.PN_arr[4],
                    self.PN_arr[5],
                )

            # right(long) of BR buoy[{6},7]
            # above(lat) line between p4[6,7] and BR buoy[6,7]
            elif self.target_set == 4:
                x = gps.latitude <= self.event_info[6] and gps.longitude <= self.SK_f(
                    gps.latitude,
                    self.PN_arr[6],
                    self.PN_arr[7],
                    self.event_info[6],
                    self.event_info[7],
                )

            # above(lat) of BR buoy[6,{7}]
            # above(lat) line between p5[8,9] and BR buoy[6,7]
            elif self.target_set == 5:
                x = gps.longitude <= self.event_info[7] and gps.longitude <= self.SK_f(
                    gps.latitude,
                    self.PN_arr[8],
                    self.PN_arr[9],
                    self.event_info[6],
                    self.event_info[7],
                )

            # above(lat) line between TL[0,1] and TR[2,3]
            elif self.target_set == 6:
                x = gps.longitude <= self.SK_f(
                    gps.latitude,
                    self.event_info[0],
                    self.event_info[1],
                    self.event_info[2],
                    self.event_info[3],
                )
            else:
                self.logging.info(
                    f"PN: ERROR: 00: TARGET SET OUT OF RANGE (1to6)\nTARGET PNT = {self.target_set}"
                )

        return x

    def PN_Perpend(self, x, c1, c2, a1, b1, a2, b2):
        # f(x) = y of line perpendicular to SK_f(a1,b1,a2,b2)

        # x:input
        # c1/c2:mid x/y
        # a1/b1:point1
        # a2/b2:point2

        # m= -m0^-1
        # v= c2-m0*c1
        m = -1 / self.SK_m(a1, b1, a2, b2)
        v = c2 - m * c1
        return m * x + v

    def PN_checkwayside(self):
        # check if sideways or upsidedown
        # [!!!!!]return self.ifupsidedown,self.ifsideways
        # self.event_info
        # PN_arr
        """
        1:rightways [standard]
        2:90deg  right
        3:180deg upsidedown
        4:90deg  left

        [4,5][10,11]
        not sideways(1/3): 3>&6< 45deg line(p3/p6)  [same, keep]
        sideways(2/4     : 3>&6< 45deg line(p3/p6)  [change long/lat asks]

        rightside up(1/2)   [same, keep]
        1:p3y<p6y
        2:p3x<p6x
        upside down(3/4)    [change >&<]
        1:p3y>p6y
        2:p3x>p6x
        """

        if (
            abs(
                self.SK_m(
                    self.PN_arr[4], self.PN_arr[5], self.PN_arr[10], self.PN_arr[11]
                )
            )
            < 1
        ):
            a = True  # sideways
        else:
            a = False

        if a:  # sideways
            if self.PN_arr[4] > self.PN_arr[10]:
                b = True  # upsidedown
            else:
                b = False  # rightside up
        else:  # rightways
            if self.PN_arr[5] > self.PN_arr[11]:
                b = True  # upsidedown
            else:
                b = False  # rightside up

        return b, a  # changed for bool table reasons to b,a


if __name__ == "__main__":
    w1 = Waypoint(42.845608, -70.977037)
    w2 = Waypoint(42.845640, -70.977002)
    w3 = Waypoint(42.845159, -70.976700)
    w4 = Waypoint(42.845775, -70.976357)
    coords = [w1, w2, w3, w4]
    eevee = Precision_Navigation(coords)
    run = True
    while run:
        try:
            waypoint = eevee.next_gps()
            if waypoint is not None:
                eevee.goToGPS(waypoint.lat, waypoint.lon)
            else:
                eevee.adjustSail(90)
        except EventFinished:
            eevee = None
            run = False
