import importlib
import math
import os
import time

import matplotlib.pyplot as plt

from sailbot import constants as c
from src.sailbot.utils import distance_between
from src.sailbot.utils import Event, EventFinished
from src.sailbot.utils.utils import Waypoint, has_reached_waypoint

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if DOCKER == "True" else False
folder = "sailbot.peripherals" if not DOCKER else "sailbot.virtualPeripherals."

camera = importlib.import_module(folder + "camera").Camera

"""
# Challenge	Goal:
    - To demonstrate the boat’s ability to autonomously locate an object
    
    # Description:
        - An orange buoy will be placed somewhere within 100 m of a reference position
        - The boat must locate, touch, and signal* such within 10 minutes of entering the search area
        - RC is not allowed after entering the search area
        - 'Signal' means white strobe on boat and/or signal to a shore station and either turn into wind or assume station-keeping mode
    
    # Scoring:
        - 15 pts max
        - 12 pts for touching (w/o signal)
        - 9 pts for passing within 1m
        - 6 pts for performing a search pattern (creeping line, expanding square, direct tracking to buoy, etc)
    
    # Assumptions: (based on guidelines)
        - 
        
    # Strategy:
        - Define a Z-shaped search pattern
        - Travel along the search path while taking panoramas to detect buoys
        
            - If x% buoy detected AND its estimated position is within search bounds:
                - Add to heatmap (combines detections that are near each other for greater accuracy)
                    
            - If heatmap has a high confidence point of interest:
                - Bookmark current position and divert course towards buoy
                - Focus camera on expected buoy position and take pictures
                
                - If that buoy is still a buoy:
                    - Keep moving towards the buoy and ram that shit
                    - Signal that boat touched the buoy when it stops getting closer
                        - NOTE: Use accelerometer instead?
                - Else:
                    - False positive (hopefully VERY rare)! PANIC! Return to previous search course
                    - Blacklist location (NOT IMPLEMENTED)
"""


class Search(Event):
    """
    Attributes:
        - search_center (Waypoint): the center of the search bounds
        - search_radius (float): the radius of the search bounds
        - event_duration (int): the length of the event in seconds
        - start_time (float): the start time of the event (seconds since 1970)

        - state (str): the search event state
            - Either 'SEARCHING', 'TRACKING', or 'RAMMING' the buoy
    """

    required_args = ["center", "search_radius"]

    def __init__(self, event_info):
        """
        Args:
            - _event_info (list): center and radius of search circle
                - expects [Waypoint(center_lat, center_long), radius]
        """
        super().__init__(event_info)

        # EVENT INFO
        self.search_center = event_info["center"]
        self.search_radius = event_info["search_radius"]
        self.event_duration = 600
        self.start_time = time.time()
        self.event_started = False

        # BOAT STATE
        self.waypoint_queue = self.create_search_pattern()

        # Boat is either SEARCHING, TRACKING or RAMMING a buoy
        self.state = "SEARCHING"

        # Used to pool together nearby detections for higher accuracy
        self.heatmap = Heatmap(chunk_radius=float(c.config["SEARCH"]["heatmap_chunk_radius"]))
        self.best_chunk = None
        self.divert_confidence_threshold = float(c.config["SEARCH"]["pooled_heatmap_confidence_threshold"])

        # Buffers for buoys that are near the edge of the search radius to account for gps estimation error
        self.search_bounds = self.search_radius + float(c.config["SEARCH"]["search_radius_tolerance"])

        # When to give up on a buoy if its no longer detected
        self.missed_consecutive_detections = 0
        self.tracking_abandon_threshold = int(c.config["SEARCH"]["tracking_abandon_threshold"])

        # When to switch from tracking to ramming mode and when to signal that the boat hit a buoy
        self.ramming_distance = int(c.config["SEARCH"]["ramming_distance"])
        self.collision_sensitivity = float(c.config["SEARCH"]["collision_sensitivity"])

        # SENSORS
        self.camera = Camera()
        self.gps = gps()
        self.transceiver = arduino(c.config["MAIN"]["ardu_port"])  # TODO: fix transceiver ref

    def next_gps(self):
        """
        Main event script logic. Executed continuously by boatMain.

        Returns either:
            - Waypoint object: The next GPS point that the boat should sail to
            - EventFinished Exception: signals that the event has been completed
        """

        # Either no buoys found yet or boat is gathering more confidence before diverting course
        if not self.event_started:
            if has_reached_waypoint(self.search_center, distance=self.search_radius):
                self.logging.info("Search event started!")
                self.start_time = time.time()
                self.waypoint_queue.pop(0)
                self.state = "SEARCHING"
            else:
                return self.waypoint_queue[0]

        if self.state == "SEARCHING":
            # Capture panorama of surroundings
            imgs = self.camera.survey(num_images=3, context=True, detect=True)

            # Error check detections & add to heatmap
            detections = 0
            for frame in imgs:
                for detection in frame.detections:
                    distance_from_center = distance_between(self.search_center, detection.GPS)

                    if distance_from_center > self.search_bounds:
                        self.logging.info(f"SEARCHING: Dropped buoy at: {detection.gps}, {distance_from_center}m from center")
                        continue

                    self.logging.info(f"SEARCHING: Buoy ({detection.conf}%) found at: {detection.GPS}")
                    self.heatmap.append(detection)
                    detections += 1

            if detections == 0:
                # No detections, continue along preset search path
                self.logging.info("SEARCHING: No buoys spotted! Continuing along search path")
                if has_reached_waypoint(self.waypoint_queue[0], distance=2):
                    self.waypoint_queue.pop(0)
                return self.waypoint_queue[0]
            else:
                # Detection! Check if boat is confident enough to move to the buoy
                self.best_chunk = self.heatmap.get_highest_confidence_chunk()

                if self.best_chunk.sum_confidence > self.divert_confidence_threshold:
                    self.logging.info(f"""SEARCHING: This bitch definitely a buoy! 
                            Bookmarking position and moving towards buoy at {self.best_chunk.average_gps}.""")
                    self.state = "TRACKING"
                    self.waypoint_queue.insert(0, self.gps)
                    self.waypoint_queue.insert(0, self.best_chunk.average_gps)
                    return self.waypoint_queue[0]

        # A buoy is found and boat is heading towards it
        elif self.state == "TRACKING":
            distance_to_buoy = distance_between(self.gps, self.waypoint_queue[0])

            if distance_to_buoy < self.ramming_distance:
                # Boat is near the buoy, TIME TO RAM THAT SHIT
                self.logging.info(f"TRACKING: {distance_to_buoy}m away from buoy! RAMMING TIME")
                self.state = "RAMMING"
            else:
                # Boat is still far away from buoy
                self.logging.info(f"TRACKING: {distance_to_buoy}m away from buoy! Closing in!")
                try:
                    self.camera.focus(self.waypoint_queue[0])
                except RuntimeError as e:
                    self.logging.warning(f"""TRACKING: Exception raised: {e}\n
                    Camera can't focus on target! Going towards last know position!""")
                    return self.waypoint_queue[0]

                frame = self.camera.capture(context=True, detect=True)

                # Abandon if boat can't find buoy multiple times in a row (hopefully VERY rare)
                if len(frame.detections) == 0:
                    self.logging.info("TRACKING: Lost buoy")
                    self.missed_consecutive_detections += 1

                    if self.missed_consecutive_detections == self.tracking_abandon_threshold:
                        self.logging.warning("TRACKING: Can't find buoy! Abandoning course and returning to search")
                        self.missed_consecutive_detections = 0
                        # self.heatmap.blacklist(self.waypoint_queue[0])
                        self.state = "SEARCHING"
                else:
                    self.missed_consecutive_detections = 0

                    for detection in frame.detections:
                        distance_from_center = distance_between(self.search_center, detection.GPS)

                        if distance_from_center > self.search_bounds:
                            self.logging.info(
                                f"TRACKING: Dropped buoy at: {detection.GPS}, {distance_from_center}m from center"
                            )
                            continue

                        self.logging.info(f"TRACKING: Buoy found at: {detection.gps}")
                        self.heatmap.append(detection)

                    self.logging.info(f"TRACKING: Continuing course to buoy at: {self.best_chunk.average_gps}")
                    self.waypoint_queue[0] = self.best_chunk.average_gps
                    return self.waypoint_queue[0]

        # Boat is very close to the buoy
        elif self.state == "RAMMING":
            # TODO: GPS is only so accurate. Use accelerometer instead!
            distance_to_buoy = distance_between(self.gps, self.waypoint_queue[0])

            if distance_to_buoy < self.collision_sensitivity:
                self.logging.info(f"Sailbot touched the buoy! Search event finished!")
                self.transceiver.send("Sailbot touched the buoy!")
                raise EventFinished

        # Times up... fuck it and assume that we touched the buoy
        if time.time() - self.start_time > self.event_duration:
            self.logging.info(f"Sailbot totally touched the buoy... Search event finished!")
            self.transceiver.send("Sailbot touched the buoy!")
            raise EventFinished

    def create_search_pattern(self, num_points=None):
        """
        Generates a zig-zag search pattern to maximimize area coverage
        Args:
            - num_points (int): how many GPS points to generate (minimum is 2 for a straight line)
                - More points means tighter search lines. Higher success chance but more distance to cover.
                - Default is automatically determined by object detections max detection distance
        Returns:
            - list[Waypoint(lat, long), ...] gps coordinates of the search pattern
        """
        # TODO: Fix whatever went wrong at competition
        # TODO: Adapt search pattern based on detection distance/windspeed/num-points
        # Metrics used to fine-tune optimal coverage
        # Camera cone of vision from
        BOAT_FOV = 242
        # Furthest distance object detection can reliably spot a buoy (m)
        MAX_DETECTION_DISTANCE = 20  # untested

        pattern = []
        for i in range(0, 30):
            if self.gps.latitude is not None:
                break
            print("waiting for gps")
        d_lat = self.gps.latitude - self.search_center.lat
        d_lon = self.gps.longitude - self.search_center.lon
        radius = math.sqrt(d_lat**2 + d_lon**2)
        ang = math.atan(d_lon / d_lat)
        ang *= 180 / math.pi

        if d_lat < 0:
            ang += 180

        tar_angs = [ang, ang + 72, ang - 72, ang - (72 * 3), ang - (72 * 2)]
        for i in range(1, 5):
            wp = Waypoint(self.search_center.lat, self.search_center.lon)
            dx = self.search_center.lon + self.search_radius * math.cos(tar_angs[i] * (math.pi / 180))
            dy = self.search_center.lat + self.search_radius * math.sin(tar_angs[i] * (math.pi / 180))
            wp.add_meters(dx, dy)
            pattern.append(wp)

        return pattern


class Heatmap:
    """Datastructure which splits the search radius into X-meter circular 'chunks'
    - Each detection has its confidence pooled with all others inside the same chunk
        - Decrease chunk radius if two separate buoys are being grouped as one
        - Increase chunk radius if the same buoy is creating multiple chunks (caused by GPS estimation error)
    - NOTE: Chunks can overlap which may cause problems (if so, then extend code to use tri/square/hex chunks instead of circles)
        - New detections will only increase the confidence of the first overlapping chunk it is contained in

    Attributes:
        - chunks (list[HeatmapChunk])
        - chunk_radius (float)
    Functions:
        - visualize(): prints out a graphical representation of the heatmap
    """

    def __init__(self, chunk_radius):
        self.chunks = []
        self.chunk_radius = chunk_radius

    def __contains__(self, detection):
        """
        Checks if a detection is inside any of the heatmap's chunks' boundary
            - Invoke using the 'in' keyword ex. 'if detection in heatmap'
        """
        for chunk in self.chunks:
            if detection in chunk:
                return True
        return False

    def append(self, detection):
        for chunk in self.chunks:
            if detection in chunk:
                chunk.append(detection)
        else:
            self.chunks.append(
                HeatmapChunk(radius=self.chunk_radius, detection=detection)
            )

    def get_highest_confidence_chunk(self):
        return max(self.chunks, key=lambda heatmap_chunk: heatmap_chunk.sum_confidence)

    def visualize(self):
        """Creates a scatterplot of the lats and lons of each chunk.
        Colors weighted by confidence value."""
        lats = []
        lons = []
        sizes = []
        confidences = []

        for chunk in self.chunks:
            lats.append(chunk.average_gps.lat)
            lons.append(chunk.average_gps.lon)
            sizes.append(chunk.radius)
            confidences.append(chunk.sum_confidence)

        fig, ax = plt.subplots()
        ax.scatter(lats, lons, s=sizes, c=confidences, cmap="magma")

        plt.title("Buoy Heatmap")
        plt.show()

        plt.savefig("test_plot")


class HeatmapChunk:
    """
    A circular boundary which combines nearby detections for better accuracy
        - Detections within the chunk's radius are assumed to be from the same buoy and averaged

    Attributes:
        - radius (float): the radial size of the chunk
        - average_gps (Waypoint): the average point between all detections within a chunk
        - detection_count (int): the number of detections within a chunk
        - sum_confidence (float): the combined total of all detections within the chunk
    """

    def __init__(self, radius, detection):
        self.radius = radius
        self.average_gps = detection.gps
        self.detection_count = 1
        self.sum_confidence = detection.conf

        self._sum_lat = detection.gps.latitude
        self._sum_lon = detection.gps.longitude

    def __contains__(self, detection):
        return distance_between(self.average_gps, detection.gps) <= self.radius

    def append(self, detection):
        self.detection_count += 1

        self._sum_lat += detection.gps.latitude
        self._sum_lon += detection.gps.longitude
        self.average_gps = Waypoint(self._sum_lat / self.detection_count, self._sum_lon / self.detection_count)

        self.sum_confidence += detection.conf
