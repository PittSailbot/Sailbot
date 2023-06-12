#from curses import KEY_B2
import math
import time
import sailbot.constants as c

import os, importlib
DOCKER = os.environ.get('IS_DOCKER', False)
DOCKER = True if DOCKER == 'True' else False
folder = "sailbot.peripherals" if not DOCKER else "sailbot.virtualPeripherals."

gps = importlib.import_module(folder + "GPS").gps


from datetime import date, datetime
from threading import Thread
from time import sleep



#============================================================================================================================
'''
next_gps() returns next long/lat (x/y) point to go to
    return ###,###:         expected result
    return None,None:       adjust sail to 90(drop it), clear target
    raise exception eventFinished:  end the event

    check inside main for a specific exception as the end of the event
    NOTE: for some events: there is no "end" of the event, must switch to manual which will clear the class in main
Events that require manual end:
    -Station Keeping
'''

#NOTE: ask tom if access gps from boat or gps class
#self.gps_class.latitude
#pass boat as parent to event class

class event:
    def __init__(self, arr):
        self.event_arr = arr
        self.totalError = 0.0
        self.oldError = 0.0
        self.oldTime = time.time()
        self.last_pnt_x, self.last_pnt_y = None,None
        self.gps_class = gps()
    
    def SK_f(self,x,a1,b1,a2,b2): return self.SK_m(a1,b1,a2,b2)*x + self.SK_v(a1,b1,a2,b2)  #f(x)=mx+b
    def SK_m(self,a1,b1,a2,b2): return (b2-b1)/(a2-a1)                                      #m: slope between two lines
    def SK_v(self,a1,b1,a2,b2): return b1-(self.SK_m(a1,b1,a2,b2)*a1)                       #b: +y between two lines
    def SK_I(self,M1,V1,M2,V2): return (V2-V1)/(M1-M2)                                      #find x-cord intersect between two lines
    def SK_d(self,a1,b1,a2,b2): return math.sqrt((a2-a1)**2 + (b2-b1)**2)                   #find distance between two points

    #TODO: find new place to relocate
    def PID(self):  #hana
        # New PID stuff
        if (time.time() - self.oldTime < 100):  # Only runs every tenth of a second #new
            # Finds the angle the boat should take
            error = self.boat_RefObj.targetAngle - self.compass.angle  # Finds how far off the boat is from its goal
            self.totalError += error  # Gets the total error to be used for the integral gain
            derivativeError = (error - self.oldError) / (
                        time.time() - self.oldtime)  # Gets the change in error for the derivative portion
            deltaAngle = c.config['CONSTANTS']["P"] * error + c.config['CONSTANTS']["I"] * self.totalError + c.config['CONSTANTS']["D"] * derivativeError  # Finds the angle the boat should be going

            # Translates the angle into lat and log so goToGPS won't ignore it
            self.boat_RefObj.currentAngle = getCoordinateADistanceAlongAngle(1000, deltaAngle + self.boat_RefObj.compass.angle)

            # Resets the variable
            self.oldTime = time.time()
            self.oldError = error
