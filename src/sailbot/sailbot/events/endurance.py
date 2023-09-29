from std_msgs.msg import String

import sailbot.constants as c
from sailbot.utils import distance_between
from sailbot.utils import Event, EventFinished
from sailbot.utils.utils import Waypoint, has_reached_waypoint

"""
# Challenge Goal:
    - To demonstrate the boat's durability and capability to sail some distance
    
    # Description:
        - The boats will sail around 4 buoys (passing within 10 m inside of buoy is OK) for up to 7 hours
    
    # Scoring:
        - 10 pts max
        - 1 pt for each 1NM lap completed autonomously (1/2 pt/lap if RC is used at any point during the lap*)
        - An additional 1pt for each continuous (no pit-stop) hr sailed; up to 6 pts
        - At least one lap must be completed to earn points
        - All boats must start each subsequent lap at the Start line following a pit stop or support boat rescue. (*No penalty for momentary RC to avoid collisions.)
    
    # Assumptions: (based on guidelines)
        - left of start direction is upstream
    
    # Strategy:
        - TODO write psuedocode about how this event logic works
"""


class Endurance(Event):
    """
    Attributes:
        - _event_info (list): 4 GPS coordinates forming a rectangle that the boat must sail around
            - expects [Waypoint(b1_lat, b1_long), Waypoint(b2_lat, b2_long), ...]
                - top left, top right, bottom left, bottom right
    """

    required_args = ["waypoint1", "waypoint2", "waypoint3", "waypoint4"]

    def __init__(self, event_info):
        super().__init__(event_info)
        self.logging.info("Endurance moment")

        # BOAT STATE
        self.waypoint_queue = [
            event_info["waypoint1"],
            event_info["waypoint2"],
            event_info["waypoint3"],
            event_info["waypoint4"],
        ]
        # TODO: ADD ROUNDING BUFFER to waypoints
        rounding_buffer = c.config["ENDURANCE"]["rounding_buffer"]

        self.gps_subscription = self._node.create_subscription(
            String, "GPS", self.ROS_GPSCallback, 10
        )

    def ROS_GPSCallback(self, data):
        string = data.data

        lat, lon, trackangle = string.replace("(", "").replace(")", "").split(",")
        currentPos = Waypoint(float(lat), float(lon))

        if distance_between(currentPos, self.waypoint_queue[0]) < float(
            c.config["CONSTANTS"]["reached_waypoint_distance"]
        ):
            self.logging.info(
                f"reached: {self.waypoint_queue[0]}, moving to: {self.waypoint_queue[1]}"
            )
            self.waypoint_queue.append(self.waypoint_queue[0])
            self.waypoint_queue.pop(0)

    def next_gps(self):
        """
        Main event script logic. Executed continuously by boatMain.

        Returns either:
            - The next GPS point that the boat should sail to stored as a Waypoint object
            - OR None to signal the boat to drop sails and clear waypoint queue
            - OR EventFinished exception to signal that the event has been completed
        """

        if has_reached_waypoint(self.waypoint_queue[0], distance=10):
            self.logging.info("Rounded buoy")
            self.waypoint_queue.pop()

        if len(self.waypoint_queue) == 0:
            raise EventFinished

        return self.waypoint_queue[0]
