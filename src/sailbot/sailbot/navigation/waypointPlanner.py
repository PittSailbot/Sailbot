import numpy as np

from sailbot.utils import boatMath, eventUtils, utils


class WaypointPlanner:
    """Manages sequential waypoint navigation and completion tracking"""

    def __init__(self, waypoint_tolerance=2.0):
        """
        Args:
            waypoint_tolerance: distance in meters where an agent is said to have completed a waypoint
        """
        self.waypoint_tolerance = waypoint_tolerance
        self.waypoints = []
        self.current_waypoint_index = 0

    @property
    def target_waypoint(self):
        if len(self.waypoints) == 0 or len(self.waypoints) <= self.current_waypoint_index:
            return None

        return self.waypoints[self.current_waypoint_index]

    @target_waypoint.setter
    def target_waypoint(self, waypoint):
        """Set the target waypoint and update current index if needed"""
        if waypoint in self.waypoints:
            self.current_waypoint_index = self.waypoints.index(waypoint)
        else:
            # If waypoint not in list, add it and set as target
            self.waypoints.append(waypoint)
            self.current_waypoint_index = len(self.waypoints) - 1

    def set_waypoint_sequence(self, waypoints):
        """Set a complete sequence of waypoints to navigate through"""
        self.waypoints = waypoints.copy()
        self.current_waypoint_index = 0

    def append_waypoint(self, waypoint):
        """Add a single waypoint to the sequence"""
        self.waypoints.append(waypoint)

    def clear(self):
        """Clear all waypoints and reset to initial state"""
        self.waypoints = []
        self.current_waypoint_index = 0
        self.target_waypoint = None

    def __str__(self):
        if self.target_waypoint is not None:
            return f"Navigating to {self.target_waypoint['name']}: ({self.current_waypoint_index}/{len(self.waypoints)} waypoints complete)"
        elif len(self.waypoints) > 0:
            return "All waypoints completed"
        else:
            return "No waypoints set"
