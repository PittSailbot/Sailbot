import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String

from sailbot import constants as c
from sailbot.navigation.waypointPlanner import WaypointPlanner
from sailbot.utils import boatMath, utils
from sailbot.utils.utils import ControlState, ImuData, Waypoint
from sailbot_interfaces.msg import WaypointQueueState


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
    SENSOR_TIMEOUT = float(c.config["NAVIGATION"]["sensor_timeout"])
    RUDDER_MIN = float(c.config["RUDDER"]["min_angle"])
    RUDDER_MAX = float(c.config["RUDDER"]["max_angle"])
    RUDDER_CENTER = (RUDDER_MAX + RUDDER_MIN) / 2

    def __init__(self):
        super().__init__("navigation")
        self.logging = self.get_logger()

        self.last_gps_time = self.get_clock().now()
        self.last_imu_time = self.get_clock().now()
        self.last_wind_time = self.get_clock().now()

        self.boat_position = Waypoint(0, 0)
        self.boat_speed = 0
        self.boat_heading = 0
        self.wind_angle = 0
        self.no_go_zone_left_bound = 0  # Left-side close-haul to wind
        self.no_go_zone_right_bound = 0  # Right-side close-haul to wind

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.queue_state_sub = self.create_subscription(
            WaypointQueueState,
            "/waypoint_queue_state",
            self.queue_state_callback,
            state_qos,
        )
        self.gps_sub = self.create_subscription(String, "/GPS", self.gps_callback, 2)
        self.speed_sub = self.create_subscription(String, "/speed", self.speed_callback, 2)
        self.imu_sub = self.create_subscription(String, "/imu", self.imu_callback, 2)
        self.windvane_sub = self.create_subscription(String, "/wind_angle", self.windvane_callback, 2)
        self.control_state_sub = self.create_subscription(String, "/control_state", self.control_state_callback, 2)

        self.cmd_sail_pub = self.create_publisher(Float32, "/cmd_sail", 10)
        self.cmd_jib_pub = self.create_publisher(Float32, "/cmd_jib", 10)
        self.cmd_rudder_pub = self.create_publisher(Float32, "/cmd_rudder", 10)

        self.nav_timer = self.create_timer(0.2, self.update_navigation)

        self.wp = WaypointPlanner()
        self.status = ""  # Additional text displayed on agent label
        self.prev_status = None

    def queue_state_callback(self, msg: WaypointQueueState):
        waypoints = [Waypoint(wp.lat, wp.lon) for wp in msg.waypoints]
        self.wp.set_waypoint_sequence(waypoints, current_waypoint_index=msg.current_index)

    def control_state_callback(self, msg: String):
        self.control_state = ControlState.fromRosMessage(msg)

    def windvane_callback(self, msg: String):
        self.last_wind_time = self.get_clock().now()
        angle = float(msg.data)
        self.wind_angle = angle % 360
        self.no_go_zone_left_bound, self.no_go_zone_right_bound = boatMath.get_no_go_zone_bounds(self.wind_angle)

    def imu_callback(self, msg: String):
        self.last_imu_time = self.get_clock().now()
        imu_data = ImuData.fromRosMessage(msg)
        self.boat_heading = (imu_data.yaw) % 360

    def gps_callback(self, msg: String):
        now = self.get_clock().now()
        new_pos = Waypoint.from_msg(msg)

        self.last_gps_time = now
        self.boat_position = new_pos

    def speed_callback(self, msg: String):
        self.boat_speed = float(msg.data)

    def __str__(self):
        if self.wp.target_waypoint is not None and self.wp.waypoints is not None:
            # Check if target_waypoint is a dictionary with "name" key
            if isinstance(self.wp.target_waypoint, dict) and "name" in self.wp.target_waypoint:
                target_name = self.wp.target_waypoint["name"]
            else:
                target_name = "waypoint"  # Fallback for non-dict targets
            return f"""Navigating to {target_name}: ({self.wp.current_waypoint_index}/{len(self.wp.waypoints)} waypoints complete)"""
        else:
            return f"""Idle: Station keeping"""

    def update_navigation(self):
        """Timer callback that checks sensors before calling the abstract tick method."""
        if self.wp.target_waypoint is None:
            self.heave_to()
            self.logging.info("Awaiting next gps", throttle_duration_sec=60)
            return
        
        now = self.get_clock().now()

        gps_stale = (now - self.last_gps_time).nanoseconds / 1e9 > 90
        imu_stale = (now - self.last_imu_time).nanoseconds / 1e9 > self.SENSOR_TIMEOUT
        wind_stale = (now - self.last_wind_time).nanoseconds / 1e9 > self.SENSOR_TIMEOUT

        if self.boat_position is None or self.boat_heading is None or self.wind_angle is None or gps_stale or imu_stale or wind_stale:
            if self.boat_position is None or gps_stale:
                self.logging.warning("No fresh GPS for autonomy to function", throttle_duration_sec=10)
            if self.boat_heading is None or imu_stale:
                self.logging.warning("No fresh IMU data for autonomy to function", throttle_duration_sec=10)
            if self.wind_angle is None or wind_stale:
                self.logging.warning("No fresh windvane for autonomy to function", throttle_duration_sec=10)
            self.heave_to()
            return
            
        self.tick()

    def tick(self):
        """Update Navigation decision-making, setting sail/jib/rudder"""
        raise NotImplementedError()

    def auto_adjust_sail(self):
        """Adjusts the sail to the optimal angle for speed"""
        # https://www.cal-sailing.org/blogfrontpage/recent-blog-posts/entry/demystifying-apparent-wind-part-1
        apparent_wind_angle = self.wind_angle if self.wind_angle <= 180 else 360 - self.wind_angle
        
        # Close hauled: ~10°, Beam reach: ~45°, Broad reach / Run: ~90°
        if apparent_wind_angle < 45:
            # Irons / Close hauled
            sail_angle = 0.0
        elif apparent_wind_angle < 90:
            # Close reach to beam reach
            sail_angle = boatMath.remap(apparent_wind_angle, 45, 90, 0.0, 50.0)
        elif apparent_wind_angle < 135:
            # Beam reach to broad reach
            sail_angle = boatMath.remap(apparent_wind_angle, 90, 135, 50.0, 80.0)
        else:
            # Broad reach to running
            sail_angle = 100.0

        msg = Float32()
        msg.data = float(sail_angle)
        self.cmd_sail_pub.publish(msg)

    def heave_to(self):
        self.cmd_sail_pub.publish(Float32(data=100.0))
        self.cmd_jib_pub.publish(Float32(data=0.0))
        self.cmd_rudder_pub.publish(Float32(data=0.0))
