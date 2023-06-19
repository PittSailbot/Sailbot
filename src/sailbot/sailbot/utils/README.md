# Utilities
Commonly used functions to make coding easier.

## Waypoints
Use the `utils.Waypoint` class to handle **all** GPS markers. This lets you reference latitude and longitude like `wp.lat` and `wp.lon` instead of `wp[0]` and `wp[1]`.
All utility functions expect to be given a Waypoint object and will not work with anything else.

## Quick function reference
### Standard libraries
- math.radians(degs): convert degrees to radians
- math.degrees(rads): convert radians to degrees
### Sailbot libraries
- boatMath.distance_between(wp1, wp2): distance in meters between two Waypoints
- boatMath.angle_between(wp1, wp2): angle from wp1 to wp2 relative to north
- Waypoint.add_meters(x, y): Updates the Waypoint by calculating new GPS from x and y meter offsets
- utils.has_reached_waypoint(wp, *optional distance threshold*): returns true if the boat is close enough to the waypoint