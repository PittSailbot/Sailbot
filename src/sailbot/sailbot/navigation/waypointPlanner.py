from typing import List

import numpy as np

from sailbot.utils import boatMath, eventUtils
from sailbot.utils.utils import Waypoint
from sailbot import constants as c


class WaypointPlanner():
    """Manages sequential waypoint navigation and completion tracking
    - Automatically assigns next waypoint (target_waypoint) when the boat reaches previous
    """

    def __init__(self):
        """
        Args:
            waypoint_tolerance: distance in meters where an agent is said to have completed a waypoint
        """
        self.waypoint_tolerance = float(c.config["CONSTANTS"]["reached_waypoint_distance"])
        self.waypoints: List[Waypoint] = []
        self.current_waypoint_index = 0

    @property
    def target_waypoint(self) -> Waypoint | None:
        if len(self.waypoints) == 0 or len(self.waypoints) <= self.current_waypoint_index:
            return None

        return self.waypoints[self.current_waypoint_index]

    @target_waypoint.setter
    def target_waypoint(self, waypoint: Waypoint):
        """Set the target waypoint and update current index if needed"""
        if waypoint in self.waypoints:
            self.current_waypoint_index = self.waypoints.index(waypoint)
        else:
            # If waypoint not in list, add it and set as target
            self.waypoints.append(waypoint)
            self.current_waypoint_index = len(self.waypoints) - 1

    def set_waypoint_sequence(self, waypoints: List[Waypoint], current_waypoint_index: int = 0):
        """Set a complete sequence of waypoints to navigate through"""
        self.waypoints = waypoints.copy()
        if len(self.waypoints) == 0:
            self.current_waypoint_index = 0
        else:
            self.current_waypoint_index = max(0, min(int(current_waypoint_index), len(self.waypoints) - 1))

    def append_waypoint(self, waypoint: Waypoint):
        """Add a single waypoint to the sequence"""
        self.waypoints.append(waypoint)

    def next_waypoint(self):
        """Move to next available waypoint, or None of no more waypoints"""
        self.current_waypoint_index += 1
        if self.current_waypoint_index >= len(self.waypoints):
            return None

    def clear(self):
        """Clear all waypoints and reset to initial state"""
        self.waypoints = []
        self.current_waypoint_index = 0

    def update_position(self, boat_position: Waypoint):
        if self.target_waypoint is not None:
            if boatMath.distance_between(boat_position, self.target_waypoint) < self.waypoint_tolerance:
                self.next_waypoint()
                return True

        return False

    def __str__(self):
        if self.target_waypoint is not None:
            return f"Navigating to {self.target_waypoint}: ({self.current_waypoint_index}/{len(self.waypoints)} waypoints complete)"
        elif len(self.waypoints) > 0:
            return "All waypoints completed"
        else:
            return "No waypoints set"
