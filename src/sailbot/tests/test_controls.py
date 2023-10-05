"""
Tests the boat's ability to move
"""
import os

import pytest

from sailbot import main

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if DOCKER == "True" else False

if not DOCKER:
    import rclpy

    rclpy.init()


@pytest.mark.skipif(DOCKER, reason="only works on raspberry pi")
def test_rudder():
    boat = main.Boat()

    boat.rudder.angle = 45
    assert 44 < boat.rudder.angle < 46, f"Incorrect rudder angle, expected 45, got: {boat.rudder.angle}"

    boat.rudder.angle = -45
    assert -46 < boat.rudder.angle < -44, f"Incorrect rudder angle, expected -45, got: {boat.rudder.angle}"

    boat.rudder.reset()
    assert -1 < boat.rudder.angle < 1, f"Incorrect rudder angle, expected 0, got: {boat.rudder.angle}"


@pytest.mark.skipif(DOCKER, reason="only works on raspberry pi")
def test_sail():
    boat = main.Boat()

    boat.sail.angle = 90
    assert 89 < boat.sail.angle < 91, f"Incorrect sail angle, expected 90, got: {boat.sail.angle}"

    boat.sail.angle = 0
    assert -1 < boat.sail.angle < 1, f"Incorrect sail angle, expected 0, got: {boat.sail.angle}"
