from rclpy.node import Node

from sailbot.events.eventUtils import Event, EventFinished, Waypoint


class ManualControl(Event):
    """
    Attributes:
        - event_info (array) - customizable arguments (unusued)
            event_info = []
    """

    required_args = []

    def __init__(self, event_info=[]):
        super().__init__(event_info)
        self.logging.info("Manual control moment")

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
