from rclpy.node import Node
import math
import time

from sailbot.events.eventUtils import Event, EventFinished, Waypoint

"""
# Challenge Goal:
    - Demonstrate a successful autonomous collision avoidance system.
        
        # Description:
            - The boat will start between two buoys
            - will sail autonomously on a reach to another buoy and return
            - Sometime during the trip, a manned boat will approach on a collision course
            - RC is not permitted after the start.
            
        # Scoring:
            - 10 pts max
            - 7 pts if the boat responds but a collision still occurs
            - 2 pt deduction if the respective buoy(s) are not reached following the avoidance maneuver
            - 3 pts max by alternative dry-land demo of appropriate sensor/rudder interaction.
            
        # Assumptions: (based on guidelines)
            - left of start direction is upstream
            - going back is harder
            
        # Strategy:
            - TODO write psuedocode about how this event logic works
"""

REQUIRED_ARGS = 2

class CollisionAvoidance(Event):
    """
    Attributes:
        - event_info (array) - buoy coordinates of path to travel
            event_info = [(start_lat, start_long), (b1_lat, b1_long)]
    """
    
    def __init__(self, event_info):
        self._node = Node('collisionAvoidanceEvent')
        self.logging = self._node.get_logger()
        if (len(event_info) != REQUIRED_ARGS):
            raise TypeError(f"Expected {REQUIRED_ARGS} arguments, got {len(event_info)}")
        
        super().__init__(event_info)
        

    def next_gps(self):
        """
        Main event script logic. Executed continuously by boatMain.
        
        Returns either:
            - The next GPS point that the boat should sail to stored as a Waypoint object
            - OR None to signal the boat to drop sails and clear waypoint queue
            - OR EventFinished exception to signal that the event has been completed
        """
        
        return Waypoint(0.0, 0.0)
    
if __name__ == "__main__":
    pass