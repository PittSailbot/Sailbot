from sailbot.navigation import navStrategy


class TackingNavigation(SailNavigation):
    def __init__(self, Dt=0.1, waypoint_tolerance=2.0, tack_duration=20.0):
        """Tacking navigation

        Args:
            Dt: time subdivision
            waypoint_tolerance: distance in meters where an agent is said to have completed a waypoint
            tack_duration: time in consecutive seconds between tacks
        """
        self.Dt = Dt
        self.current_tack = 1  # 1 for starboard, -1 for port
        self.tack_timer = 0.0
        self.tack_duration = tack_duration
        self.tack_angle = 45.0  # nearest sail direction from oncoming wind

        super().__init__(waypoint_tolerance)

    def tick(self, agent, true_wind):
        """Update Navigation decision-making"""
        if self.wp.target_waypoint is None:
            # Maintain current position
            if len(self.wp.waypoints) == 0:
                self.wp.target_waypoint = agent.pos
            else:
                self.wp.target_waypoint = self.wp.waypoints[-1]
            # self.station_keep(agent, true_wind)
        else:
            self.navigate_to_waypoint(agent, true_wind)

    def calculate_heading(self, agent, true_wind):
        self.wp.update(agent.pos)

        if self.wp.target_waypoint is None:
            return

        # Handle both dictionary waypoints and array positions
        if isinstance(self.wp.target_waypoint, dict):
            target_x = self.wp.target_waypoint["x"]
            target_y = self.wp.target_waypoint["y"]
        else:
            # Assume it's an array-like object [x, y, ...]
            target_x = self.wp.target_waypoint[0]
            target_y = self.wp.target_waypoint[1]

        dx = target_x - agent.pos[0]
        dy = target_y - agent.pos[1]
        desired_heading = np.rad2deg(np.arctan2(dy, dx))

        aw_mag, aw_dir = relative_wind(agent, true_wind)
        # Convert [0, 360] to [0, 180] where 0 = downwind, 180=headwind, 90=perpendicular (of agent's current heading)
        relative_wind_angle = aw_dir
        if relative_wind_angle > 180:
            relative_wind_angle = abs(relative_wind_angle - 360)
        relative_wind_angle = relative_wind_angle % 180

        # [0, 180] (of agent's desired heading angle relative to wind)
        # wind_to_desired_angle = (relative_wind_angle - desired_heading) % 180
        wind_direction = np.rad2deg(np.arctan2(true_wind[1], true_wind[0])) % 360
        wind_to_desired_angle = abs((desired_heading - wind_direction + 180) % 360 - 180)

        # Check if sailing upwind (trying to sail within 45deg of upwind direction)
        # Only tack if trying to sail into the wind (upwind), not with the wind (downwind)
        upwind_direction = (wind_direction + 180) % 360  # Direction wind is coming FROM
        upwind_angle = abs((desired_heading - upwind_direction + 180) % 360 - 180)

        if upwind_angle < self.tack_angle:  # Too close to upwind, need to tack
            # Calculate both possible tack headings relative to upwind direction
            port_tack = (upwind_direction + self.tack_angle) % 360
            starboard_tack = (upwind_direction - self.tack_angle) % 360

            # Calculate which tack gets us closer to desired heading
            port_diff = abs((port_tack - desired_heading + 180) % 360 - 180)
            starboard_diff = abs((starboard_tack - desired_heading + 180) % 360 - 180)

            # Choose the better tack and update current tack state
            if port_diff < starboard_diff:
                self.current_tack = 1  # Port tack
                chosen_heading = port_tack
                tack_name = "PORT"
            else:
                self.current_tack = -1  # Starboard tack
                chosen_heading = starboard_tack
                tack_name = "STAR"

            self.status = f"TACK {tack_name} -> {chosen_heading:.0f}° (target: {desired_heading:.0f}°)"
            return chosen_heading
        else:
            # Can sail more directly towards waypoint
            self.status = f"DIRECT -> {desired_heading:.0f}°"
            return desired_heading

    def navigate_to_waypoint(self, agent, true_wind):
        """
        Navigate towards the target waypoint considering wind direction with proper tacking

        Args:
            true_wind: np.array([wind_north, wind_east]) in m/s
        """
        self.wp.update(agent.pos)

        if self.wp.target_waypoint is None:
            return

        desired_heading = self.calculate_heading(agent, true_wind)
        agent.cmd_heading = desired_heading % 360
