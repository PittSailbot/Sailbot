from sailbot.navigation.waypointPlanner import WaypointPlanner


class SailNavigation:
    """Navigation behavior tree for waypoint following, station-keeping, etc."""

    def __init__(self, waypoint_tolerance=2.0):
        """Navigation system using composition with WaypointPlanner

        Args:
            waypoint_tolerance: distance in meters where an agent is said to have completed a waypoint
        """
        self.wp = WaypointPlanner(waypoint_tolerance)
        self.status = ""  # Additional text displayed on agent label

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

    def tick(self, agent, true_wind):
        """Update Navigation decision-making"""
        raise NotImplementedError()

    def set_waypoint_sequence(self, waypoints):
        """Set a sequence of waypoints to navigate through"""
        self.wp.set_waypoint_sequence(waypoints)

    def navigate_to_waypoint(self, agent, true_wind):
        """
        Navigate towards the target waypoint considering wind direction with proper tacking

        Args:
            true_wind: np.array([wind_north, wind_east]) in m/s
        """
        self.wp.update(agent.pos)

        if self.wp.target_waypoint is None:
            return

        return
