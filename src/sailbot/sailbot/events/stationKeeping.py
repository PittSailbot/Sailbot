import math
import time

from src.sailbot.sailbot.utils.eventUtils import Event, Waypoint, EventFinished
import src.sailbot.sailbot.constants as c

import os, importlib

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if DOCKER == "True" else False
folder = "sailbot.peripherals" if not DOCKER else "sailbot.virtualPeripherals."

windVane = importlib.import_module(folder + "windvane").windVane
gps = importlib.import_module(folder + "GPS").gps

"""
# Challenge	Goal:
            - To demonstrate	the	ability	of the boat to remain close to one position and respond to time-based commands.	
        
        # Description:
            - The boat will enter a 40 x 40m box and attempt to stay inside the box for 5 minutes.
            - It must then exit within 30 seconds to avoid a penalty.
            
        # Scoring:
            - 10 pts max
            - 2 pts per minute within the box during the 5 minute test (the boat may exit and reenter multiple times).
            - 2 pts per minute will be deducted for time within the box after 5½ minutes.	
            - The final score will be reduced by 50% if any RC is preformed from the start of the 5 minute event	until the boat’s final exit.
            - The final score will be to X.X precision
            
        # Assumptions: (based on guidelines)
            - front is upstream
            
        # Strategy:
            - 1.) wait till fall behind 80%
            - 2.) sail to 90%, until at 90%
            - 3.) set sail flat
            - 4.) if behind 75%, go to step 2, repeat
            - 5.) GTFO (find&sail to best point) after time limit
                - DO NOT JUST DROP SAIL
                - how we won event first time was dropping sail
                - and floating from front to end for total of 5 minute duration travel
"""


class StationKeeping(Event):
    """
    Attributes:
        - event_info (list): 4 GPS coordinates forming a 40m^2 rectangle that the boat must remain in
            - expects [Waypoint(b1_lat, b1_long), Waypoint(b2_lat, b2_long), ...]
                - top left, top right, bottom left, bottom right
    """

    REQUIRED_ARGS = 4

    def __init__(self, event_info):
        super().__init__(event_info)

        """#see SK_perc_guide() notes on calculating go-to points
        #running:
        #1.) wait till fall behind 80%
        #2.) sail to 90%, until at 90%
        #3.) set sail flat
        #4.) if behind 75%, go to step 2, repeat
        #5.) GTFO (find&sail to best point) after time limit
            #DO NOT JUST DROP SAIL
                #how we won event first time was dropping sail
                #and floating from front to end for total of 5 minute duration travel"""
        self.time_perc = 5 * 60 * (70 / 100)  # time to leave, 5 minute limit * %

        # ALGO ARRAY========================================
        # send in wanted %s in desmos calculation, and return what they are (m,b in y=mx+b; or x,y cord of center of box on %'s line)
        type_inpArr = [0, 0, 0, 1]  # m,b or x,y
        perc_inpArr = [80, 75, 90, 90]  # %'s
        self.cool_arr = self.SK_perc_guide(perc_inpArr, type_inpArr, self.event_info)
        del type_inpArr, perc_inpArr
        # whats contained in cool_arr:
        # (0,1)80-line,      (2,3)75-line,
        # (4,5)90-line,      (6,7)90-point,

        # [always auto put on end of cool_arr thats not caluclated from input]:
        # (8,9)Front-line (here cause of cart_perimiter_scan),
        # (10,11)Left-line,  (12,13)Right-line,
        # (14,15)Back-line
        # (16) mid m line for line check

        # other Algo sets========================================
        self.start = True
        self.escape_x, self.escape_y = (
            None,
            None,
        )  # determining best out to go to to leave box based on angle of run (wind)
        self.skip = False  # faster if statement for time; holdout from prev notation

        # time calc
        self.start_time = time.time()

    def next_gps(self):
        # time based checks, off-set the set GPS========================================
        curr_time = time.time()
        # gtfo, times up
        if self.skip or curr_time - self.start_time >= self.time_perc:
            # find best point to leave:
            if self.escape_x == None:
                self.skip = True  # faster if statement
                self.escape_x, self.escape_y = self.cart_perimiter_scan(
                    self.cool_arr[-7:-1]
                )  # i thought the func name sounded cool
            self.last_pnt_x, self.last_pnt_y = self.escape_x, self.escape_y
            return Waypoint(self.escape_x, self.escape_y)

        # if not in box========================================
        # ordered in certain way of most importance, handle up/down first before too left or right
        # also put before time because then it doesnt matter cause it's already out

        # past front line of box
        if not (self.SK_line_check(self.cool_arr[-9:-7], self.cool_arr[-3:-1], self.cool_arr[-1])):
            self.logging.info("too forward")
            # loosen sail, do nuthin; drift
            # .adjustSail(90)
            self.last_pnt_x, self.last_pnt_y = None, None
            return None

        # past bottom line of box
        elif not (self.SK_line_check(self.cool_arr[-3:-1], self.cool_arr[-9:-7], self.cool_arr[-1])):
            self.logging.info("too back")
            # go to 90deg line
            self.last_pnt_x, self.last_pnt_y = self.cool_arr[6], self.cool_arr[7]
            return Waypoint(self.cool_arr[6], self.cool_arr[7])

        # past left line of box
        elif not (self.SK_line_check(self.cool_arr[-7:-5], self.cool_arr[-5:-3], self.cool_arr[-1])):
            self.logging.info("too left")
            # find/go-to intersect of line (+)35degrees of wind direction to left line
            # mini cart scan
            t_x, t_y = self.mini_cart_perimiter_scan(self.cool_arr[-7:-5], "L")
            self.last_pnt_x, self.last_pnt_y = t_x, t_y
            return Waypoint(t_x, t_y)

        # past right line of box
        elif not (self.SK_line_check(self.cool_arr[-5:-3], self.cool_arr[-7:-5], self.cool_arr[-1])):
            self.logging.info("too right")
            # find/go-to intersect of line (-)35degrees of wind direction to left line
            # mini cart scan
            t_x, t_y = self.mini_cart_perimiter_scan(self.cool_arr[-5:-3], "R")
            self.last_pnt_x, self.last_pnt_y = t_x, t_y
            return Waypoint(t_x, t_y)

        # passed checks: SAILING; DOING THE EVENT========================================
        # beginning set up
        if self.start:
            self.logging.info("start if")
            # if not moving and behind 80%
            if self.SK_line_check(self.cool_arr[0:2], self.cool_arr[-3:-1], self.cool_arr[-1]):
                self.logging.info("start: behind 80%; ending start")
                self.start = False
                self.last_pnt_x, self.last_pnt_y = self.cool_arr[6], self.cool_arr[7]
                return Waypoint(self.cool_arr[6], self.cool_arr[7])  # go to 90deg line
            # if this doesnt pass, its WITHIN BOX but is ahead of 80; so it returns init'd last_pnt which is to loosen sail and drift (WHAT WE WANT)
            else:
                self.logging.info("start: ahead 80%")

        # majority of the sail
        else:
            # if behind 75%:sail back
            if self.SK_line_check(self.cool_arr[2:4], self.cool_arr[-3:-1], self.cool_arr[-1]):
                self.last_pnt_x, self.last_pnt_y = self.cool_arr[6], self.cool_arr[7]
                return Waypoint(self.cool_arr[6], self.cool_arr[7])  # go to 90deg line

            # if past or at 90% (redundence reduction)
            elif not (self.SK_line_check(self.cool_arr[4:6], self.cool_arr[-3:-1], self.cool_arr[-1])):
                self.last_pnt_x, self.last_pnt_y = None, None
                return None  # loosen sail, do nuthin

        return Waypoint(self.last_pnt_x, self.last_pnt_y)  # fall back return if nested if's dont pass

    # give %-line of box and other lines(details in SK) used in algo
    def SK_perc_guide(self, inp_arr, type_arr, buoy_arr):
        """#calc front/back/sides mid point
        #find the parameter lat/long value per percent
        #calc line 75%/80%/90% (give long, if lat) towards front between them
            #input an array of wanted %'s, return array with matching x/y's (in own array, array of arrays)
                #saves on calc times
            #https://www.desmos.com/calculator/yjeqtqunbh
                #go with s scaling
                    #1.)find midpoints
                    #2.)between mid1, mid3: perc scale x/y's
                    #works no matter rotation of boat
                #last in inp_arr returns x/y point that is % way of the box
                    #rest is m/b's
                #add m/b of back line, at end of ret_arr
                #add m of front/back midpoint line, at end of ret_arr
                #0: m/b, 1: x/y
        #then add other details need to the very end"""

        ret_arr = []
        mid_arr = []  # , m_arr=[], b_arr=[]

        # midpoints ==========================
        # annoying optimization work; done with skips in 'i' indexing
        """
        - i=x cord of buoy, even index in array
        - need to find with different combos of all the buoy coords to find the point in the middle of them
        - organize those combos with a FAST FOR STATEMENT using index skipping
        - can see depreciated "straightforward" way of doing it below, this looks nicer (and takes less memory?)
        - need to find these midpoints to calcualte the perc lines/points given in input of function
        """
        #   (12,13,34,24);(front,left,back,right)
        #   02,04,46,26
        a = [0, 2, 0, 4, 4, 6, 2, 6]  # optimizing code with for rather then long ass list
        # 0,1, 2,3, 4,5, 6,7
        for i in range(4):  # 0,1,2,3
            # TODO: remove nice variables: just fill in and make two lines (optimization)
            # NOTE: nah, fuck that^^^
            j1 = a[i * 2]  # 0 - 0 - 4 - 2
            k1 = j1 + 1  # 1 - 1 - 5 - 3
            j2 = a[(i * 2) + 1]  # 2 - 4 - 6 - 6  next over in "a"
            k2 = j2 + 1  # 3 - 5 - 7 - 7

            p = (buoy_arr[j1] + buoy_arr[j2]) / 2  # 0+1/2: j1,j2
            mid_arr.append(p)  # p
            mid_arr.append(self.SK_f(p, buoy_arr[j1], buoy_arr[k1], buoy_arr[j2], buoy_arr[k2]))  # p,j1,k1,j2,k2

        """
        #DEPRECIATED
        #DO NOT DELETE, USE FOR EXPLANATION FOR OPTIMIZATION
        # m's and b's ==========================
        #   dont wanna just delete cause dont wanna rewrite if somehow need them
        #   (mid13,mid24; 1,2; 3,4; mid12,mid34)
        m_arr.append(self.SK_m(mid_arr[2], mid_arr[3], mid_arr[6], mid_arr[7]))  # mid13 - mid24 (2,4)  m2
        #m_arr.append(self.SK_m(buoy_arr[0], buoy_arr[1], buoy_arr[2], buoy_arr[3]))  # 1 - 2            front
        m_arr.append(self.SK_m(buoy_arr[4], buoy_arr[5], buoy_arr[6], buoy_arr[7]))  # 3 - 4            back
        #m_arr.append(self.SK_m(mid_arr[0], mid_arr[1], mid_arr[4], mid_arr[5]))  # mid12 - mid34 (1,3)  down center

        b_arr.append(self.SK_v(mid_arr[2], mid_arr[3], mid_arr[6], mid_arr[7]))  # mid13 - mid24 (2,4)
        #b_arr.append(self.SK_v(buoy_arr[0], buoy_arr[1], buoy_arr[2], buoy_arr[3]))  # 1 - 2
        b_arr.append(self.SK_v(buoy_arr[4], buoy_arr[5], buoy_arr[6], buoy_arr[7]))  # 3 - 4
        #b_arr.append(self.SK_v(mid_arr[0], mid_arr[1], mid_arr[4], mid_arr[5]))  # mid12 - mid34 (1,3)


        #front/back mid line for facing use
        m_arr.append(self.SK_m(mid_arr[0],mid_arr[1],mid_arr[4],mid_arr[5]))"""
        m2 = self.SK_m(mid_arr[2], mid_arr[3], mid_arr[6], mid_arr[7])  # slope between side line's midpoints

        # newline: s-scale
        # adding all perc line/point into return array (from input in function)
        for i in range(len(inp_arr)):
            perc = inp_arr[i] / 100
            x = perc * mid_arr[0] + (1 - perc) * mid_arr[4]
            y = perc * mid_arr[1] + (1 - perc) * mid_arr[5]

            if type_arr[i] == 1:  # x/y
                ret_arr.append(x)
                ret_arr.append(y)
                continue
            else:
                ret_arr.append(m2[0])  # m
                ret_arr.append(y - m2[0] * x)  # b

        # sides-line for cart_perimiter_scan; additional additions to return
        # front
        ret_arr.append(self.SK_m(buoy_arr[0], buoy_arr[1], buoy_arr[2], buoy_arr[3]))  # m buoy1,buoy2
        ret_arr.append(self.SK_v(buoy_arr[0], buoy_arr[1], buoy_arr[2], buoy_arr[3]))  # b buoy1,buoy2
        # left
        ret_arr.append(self.SK_m(buoy_arr[0], buoy_arr[1], buoy_arr[4], buoy_arr[5]))  # m buoy1,buoy3
        ret_arr.append(self.SK_v(buoy_arr[0], buoy_arr[1], buoy_arr[4], buoy_arr[5]))  # b buoy1,buoy3
        # right
        ret_arr.append(self.SK_v(buoy_arr[2], buoy_arr[3], buoy_arr[6], buoy_arr[7]))  # m buoy2,buoy4
        ret_arr.append(self.SK_v(buoy_arr[2], buoy_arr[3], buoy_arr[6], buoy_arr[7]))  # b buoy2,buoy4
        """#back-line, m of middle-line(linecheck); additional additions to return
        ret_arr.append(m_arr[1])
        ret_arr.append(b_arr[1])
        ret_arr.append(m_arr[2])
        #ret_arr.append(b_arr[2])"""
        # back
        ret_arr.append(self.SK_m(buoy_arr[4], buoy_arr[5], buoy_arr[6], buoy_arr[7]))  # m buoy3,buoy4
        ret_arr.append(self.SK_v(buoy_arr[4], buoy_arr[5], buoy_arr[6], buoy_arr[7]))  # b buoy3,buoy4
        ret_arr.append(self.SK_m(mid_arr[0], mid_arr[1], mid_arr[4], mid_arr[5]))

        return ret_arr

    # if past line
    def SK_line_check(self, Tarr, Barr, mid_m):
        # TRUE: BEHIND LINE
        # FALSE: AT OR PAST LINE

        # Ix/y:  current location of boat
        #       gps.longitude, gps.latitude
        # Tarr:  m/b compare line
        # arr:   m/b Back line,

        # Fa:front
        # Fb:mid
        # Fc:back

        Fa = 0
        Fb = 0
        Fc = 0  # temp sets
        # check if sideways =========================
        # input x/y as Buoy x/y's to func
        gps.updategps()
        if self.DEBUG:
            self.gps_spoof()
        if abs(mid_m) < 1:  # Barr is secretly the mid m line shhhhhhh (LOOK AT ME)
            # sideways  -------------------
            # x=(y-b)/m
            Fa = (gps.latitude - Tarr[1]) / Tarr[0]
            Fb = gps.longitude
            Fc = (gps.latitude - Barr[1]) / Barr[0]
        else:
            # rightways  -------------------
            # y=mx+b
            Fa = Tarr[0] * gps.longitude + Tarr[1]
            Fb = gps.latitude
            Fc = Barr[0] * gps.longitude + Barr[1]

        if Fa > Fc:  # upright
            if Fa >= Fb:
                return False  # past or equal
            else:
                return True  # behind
        else:  # upside down
            if Fa <= Fb:
                return False  # past or equal
            else:
                return True  # behind

    # find best point of run to leave box
    def cart_perimiter_scan(self, arr):
        # DETAILS
        """#https://www.desmos.com/calculator/rz8tfc8fwn
        #see what mid point closest (Left,Back,Right)
            #cartesian with rand radius
                #find point at perimeter at -45 or 125 (left,right) degrees (LDeg,RDeg line)
                #find m/b of both
                #x = r * cos( θ )
                #y = r * sin( θ );  r=5(doesnt matter)
            #take I() of LDeg,LSide; LDeg,BSide; RDeg,RSide; RDeg,BSide
                #find closest, sail to

        #arr: back-line,left-line,right-line (m,b's) 01,23,45
            #find x,y's of degrees at best run points left and right"""

        # STRAIGHTFORWARD EXPLANATION:
        # make a point in the 2 best directions of run using cart math
        # make a line between the boat and both points
        # find intersection of those two lines and the boat
        # determine with intersection is closest

        gps.updategps()
        if self.DEBUG:
            self.gps_spoof()
        lat = gps.latitude
        long = gps.longitude
        t = math.pi / 180
        o = windVane.position
        r = (5 / 6371000) * (180 / math.pi)
        lx = r * math.cos(135 * t + o * t) + lat  # left side run point
        ly = r * math.sin(135 * t + o * t) + long
        rx = r * math.cos(-135 * t + o * t) + lat  # right side run point
        ry = r * math.sin(-135 * t + o * t) + long
        # into m,b's
        lm = self.SK_m(lx, ly, lat, long)
        lb = self.SK_v(lx, ly, lat, long)
        rm = self.SK_m(rx, ry, lat, long)
        rb = self.SK_v(rx, ry, lat, long)
        # del t,o,lx,ly,rx,ry

        # find intersects of LDeg,LSide; LDeg,BSide; RDeg,RSide; RDeg,BSide
        t_arr = []
        t_arr.append(self.SK_I(lm, lb, arr[2], arr[3]))  # x1(0)
        t_arr.append(lm * t_arr[0] + lb)  # y1(1)
        t_arr.append(self.SK_I(lm, lb, arr[0], arr[1]))  # x2(2)
        t_arr.append(lm * t_arr[2] + lb)  # y2(3)
        t_arr.append(self.SK_I(rm, rb, arr[4], arr[5]))  # x3(4)
        t_arr.append(rm * t_arr[4] + rb)  # y3(5)
        t_arr.append(self.SK_I(rm, rb, arr[0], arr[1]))  # x4(6)
        t_arr.append(rm * t_arr[6] + rb)  # y4(7)

        # use distance equation and find closest
        sd = self.SK_d(t_arr[0], t_arr[1], lat, long)
        si = -1
        for i in range(3):
            a = self.SK_d(t_arr[2 * (i + 1)], t_arr[(2 * i) + 3], lat, long)  # skip 0,1
            if a < sd:
                sd = a
                si = i
        return t_arr[si + 1], t_arr[si + 2]

    # same concept as cart_perm_scan
    # used when OUTSIDE BOX to find best line to attack INTO BOX
    # just for when left/right of box
    def mini_cart_perimiter_scan(self, arr, case):
        gps.updategps()
        if self.DEBUG:
            self.gps_spoof()
        lat = gps.latitude
        long = gps.longitude
        t = math.pi / 180
        o = windVane.position
        r = (5 / 6371000) * (180 / math.pi)

        if case == "L":
            x = r * math.cos(55 * t + o * t) + lat  # +35 from windvane
            y = r * math.sin(55 * t + o * t) + long
        elif case == "R":
            x = r * math.cos(125 * t + o * t) + lat  # -35
            y = r * math.sin(125 * t + o * t) + long
        else:
            raise TypeError("mini_cart_perimiter_scan ERROR")

        m = self.SK_m(x, y, lat, long)
        b = self.SK_v(x, y, lat, long)

        ret1 = self.SK_I(arr[0], arr[1], m, b)
        return ret1, m * ret1 + b

    def SK_f(self, x, a1, b1, a2, b2):
        return self.SK_m(a1, b1, a2, b2) * x + self.SK_v(a1, b1, a2, b2)  # f(x)=mx+b

    def SK_m(self, a1, b1, a2, b2):
        return (b2 - b1) / (a2 - a1)  # m: slope between two lines

    def SK_v(self, a1, b1, a2, b2):
        return b1 - (self.SK_m(a1, b1, a2, b2) * a1)  # b: +y between two lines

    def SK_I(self, M1, V1, M2, V2):
        return (V2 - V1) / (M1 - M2)  # find x-cord intersect between two lines

    def SK_d(self, a1, b1, a2, b2):
        return math.sqrt((a2 - a1) ** 2 + (b2 - b1) ** 2)  # find distance between two points


if __name__ == "__main__":
    pass
