"""
Extended compilation and initilization tests
"""
import os

import pytest

from sailbot import main
from sailbot import constants as c

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if DOCKER == "True" else False

if not DOCKER:
    import rclpy
    from std_msgs.msg import String

    rclpy.init()


@pytest.mark.skipif(DOCKER, reason="only works on raspberry pi")
def test_main_init():
    boat = main.Boat()


@pytest.mark.skipif(DOCKER, reason="only works on raspberry pi")
def test_event_init():
    boat = main.Boat(calibrateOdrive=False)

    _node = rclpy.Node()
    _node.create_publisher(String, "mode", 10)
    modes = [
        c.config["MODES"]["MOD_PRECISION_NAVIGATE"],
        c.config["MODES"]["MOD_ENDURANCE"],
        c.config["MODES"]["MOD_STATION_KEEPING"],
        c.config["MODES"]["MOD_SEARCH"],
    ]
    for mode in modes:
        msg = String()
        msg.data = f"Set Mode:{mode}"
        _node.publish(msg)
        rclpy.spin_once(boat)
        assert boat.modeSetting == mode