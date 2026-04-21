import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from sailbot import constants as c
from sailbot.navigation.waypointPlanner import WaypointPlanner
from sailbot.utils import boatMath, utils
from sailbot.utils.utils import ControlState, ImuData, Waypoint


class NavigationStrategy(Node):
    """Autonomously navigates the boat to a desired GPS waypoint
    Listens to:
        - /next_gps (Waypoint): The next GPS point to navigate to
        - /gps (String):
        - /compass (String):
        - /windvane (String):
        - /control_state (Bool):
    Publishes to:
        - /cmd_rudder (String)
        - /cmd_sail (String)
    """

    ACCEPTABLE_ERROR = float(c.config["RUDDER"]["acceptable_error"])
    SMOOTHING_CONSTANT = float(c.config["RUDDER"]["smooth_const"])
    RUDDER_MIN = float(c.config["RUDDER"]["min_angle"])
    RUDDER_MAX = float(c.config["RUDDER"]["max_angle"])
    RUDDER_CENTER = (RUDDER_MAX - RUDDER_MIN) / 2

    def __init__(self, waypoint_tolerance=2.0):
        """
        Args:
            waypoint_tolerance: distance in meters where an agent is said to have completed a waypoint
        """
        super().__init__("navigation")
        self.logging = self.get_logger()

        self.position = Waypoint(0, 0)  # Position of the boat
        self.compass_angle = 0  # Heading of the boat
        self.wind_angle = 0
        self.no_go_zone_left_bound = 0  # Left-side close-haul to wind
        self.no_go_zone_right_bound = 0  # Right-side close-haul to wind

        self.next_gps_sub = self.create_subscription(String, "/next_gps", self.next_gps_callback, 2)
        self.gps_sub = self.create_subscription(String, "/GPS", self.gps_callback, 2)
        self.imu_sub = self.create_subscription(String, "imu", self.imu_callback, 2)
        self.windvane_sub = self.create_subscription(String, "/wind_angle", self.windvane_callback, 2)
        self.control_state_sub = self.create_subscription(String, "/control_state", self.control_state_callback, 2)

        self.cmd_sail_pub = self.create_publisher(Float32, "/cmd_sail", 10)
        self.cmd_jib_pub = self.create_publisher(Float32, "/cmd_jib", 10)
        self.cmd_rudder_pub = self.create_publisher(Float32, "/cmd_rudder", 10)

        self.nav_timer = self.create_timer(0.2, self.tick)

        self.wp = WaypointPlanner(waypoint_tolerance)
        self.status = ""  # Additional text displayed on agent label

    def next_gps_callback(self, msg):
        next_gps = Waypoint.from_msg(msg)
        if next_gps != self.latest_waypoint:
            self.logging.info(f"Navigating to {next_gps}")
            self.latest_waypoint = next_gps

    def control_state_callback(self, msg):
        self.control_state = ControlState.fromRosMessage(msg)

    def windvane_callback(self, msg):
        angle = float(msg.data)
        self.wind_angle = angle % 360
        self.no_go_zone_left_bound, self.no_go_zone_right_bound = boatMath.get_no_go_zone_bounds(self.wind_angle, self.compass_angle)

    def imu_callback(self, msg):
        imu_data = ImuData.fromRosMessage(msg)
        self.compass_angle = (imu_data.yaw) % 360
        self.no_go_zone_left_bound, self.no_go_zone_right_bound = boatMath.get_no_go_zone_bounds(self.wind_angle, self.compass_angle)

    def gps_callback(self, msg):
        self.position = Waypoint.from_msg(msg)

    def __str__(self):
        if self.wp.target_waypoint is not None and self.wp.waypoints is not None:
            # Check if target_waypoint is a dictionary with "name" key
            if isinstance(self.wp.target_waypoint, dict) and "name" in self.wp.target_waypoint:
                target_name = self.wp.target_waypoint["name"]
            else:
                target_name = "Position"  # Fallback for non-dict targets
            return f"""Navigating to {target_name}: ({self.wp.current_waypoint_index}/{len(self.wp.waypoints)} waypoints complete)"""
        else:
            return f"""Idle: Station keeping"""

    def tick(self):
        """Update Navigation decision-making, setting sail/jib/rudder"""
        raise NotImplementedError()

    def auto_adjust_sail(self):
        """Adjusts the sail to the optimal angle for speed"""
        # TODO: Test & validate

        if self.wind_angle > 180:
            wind_angle = 180 - (self.wind_angle - 180)
        else:
            wind_angle = self.wind_angle

        sail_angle = max(min(wind_angle / 2, 90), 3)

        msg = Float32()
        msg.data = float(sail_angle)
        self.cmd_sail_pub.publish(msg)
