from rclpy.node import Node
import math
import time

from src.sailbot.sailbot.utils.eventUtils import Event, EventFinished, Waypoint
from src.sailbot.sailbot.peripherals.windvane import WindVane
from src.sailbot.sailbot.peripherals.GPS import gps

import os, importlib

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if DOCKER == "True" else False
folder = "sailbot.peripherals" if not DOCKER else "sailbot.virtualPeripherals."

windVane = importlib.import_module(folder + "windvane").windVane
gps = importlib.import_module(folder + "GPS").gps

"""
# Challenge	Goal:
        - To demonstrate the ability of the boat to remain close to one position and respond to time-based commands.	

        # Description:
            - The boat will enter a 40 x 40m box and attempt to stay inside the box for 5 minutes.
            - It must then exit within 30 seconds to avoid a penalty.

        # Scoring:
            - 10 pts max
            - 2 pts per minute within the box during the 5 minute test (the boat may exit and reenter multiple times).
            - 2 pts per minute will be deducted for time within the box after 5½ minutes.	
            - The final score will be reduced by 50% if any RC is preformed from the start of the 5 minute event until the boat’s final exit.
            - The final score will be to X.X precision

        # Strategy:
            - Keep moving towards center until its time to leave
            - Turn in the direction of the wind and bail
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

        # EVENT INFO
        self.bounds = Bounds(event_info)
        self.start_time = time.time()
        self.leave_time = self.start_time + 4.25 * 60
        self.event_started = False

        # BOAT STATE
        self.waypoint_queue = []

        # SENSORS
        self.gps = gps.gps
        self.windvane = WindVane()

    def next_gps(self):
        """
        Main event script logic. Executed continuously by boatMain.

        Returns either:
            - The next GPS point that the boat should sail to stored as a Waypoint object
            - OR None to signal the boat to drop sails and clear waypoint queue
            - OR EventFinished exception to signal that the event has been completed
        """

        # Move to bounds of event
        if not self.event_started:
            if Waypoint(self.gps.gps.latitude, self.gps.gps.longitude) in self.bounds:
                self.logging.info("Boat has entered the station keeping bounds!")
                self.event_started = True
                self.start_time = time.time()
            else:
                self.logging.debug("Moving to event bounds")
                return self.bounds.center

        # Stay inside center of bounds
        if time.time() < self.leave_time:
            return self.bounds.center

        # Turn to downwind and gtfo
        self.logging.debug("Leaving event bounds")
        downwind_angle = math.radians(abs(self.windvane.angle + 180) % 360)

        escape_point = self.gps.gps
        escape_point.add_meters(30 * math.cos(downwind_angle), 30 * math.sin(downwind_angle))
        return escape_point


class Bounds:
    """Rectangular bounding box for the station keeping event"""

    def __init__(self, event_info):
        self.top_left = event_info[0]
        self.top_right = event_info[1]
        self.bottom_left = event_info[2]
        self.bottom_right = event_info[3]
        self.center = self.calculate_center_gps()

    def __contains__(self, waypoint):
        """Returns true if a gps waypoint is inside the bounds"""
        latitudes = [self.top_left.lat, self.top_right.lat, self.bottom_left.lat, self.bottom_right.lat]
        longitudes = [self.top_left.lon, self.top_right.lon, self.bottom_left.lon, self.bottom_right.lon]

        return min(latitudes) <= waypoint.lat <= max(latitudes) and min(longitudes) <= waypoint.lon <= max(longitudes)

    def calculate_center_gps(self):
        """Calculates the center GPS point of the square bounds"""
        sum_lat = self.top_left.lat + self.top_right.lat + self.bottom_left.lat + self.bottom_right.lat
        sum_lon = self.top_left.lon + self.top_right.lon + self.bottom_left.lon + self.bottom_right.lon

        return Waypoint(sum_lat / 4, sum_lon / 4)
