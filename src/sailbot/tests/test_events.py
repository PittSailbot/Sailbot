"""
Tests event compilation and functionality
"""
import os

import pytest

from sailbot import main
from sailbot.events import endurance, precisionNavigation, search, stationKeeping

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if DOCKER == "True" else False

if not DOCKER:
    import rclpy

    # from sailbot.peripherals import compass, GPS, transceiver, windvane

    rclpy.init()


@pytest.mark.skip(reason="Not implemented")
@pytest.mark.skipif(DOCKER, reason="only works on raspberry pi")
def test_endurance():
    pass


@pytest.mark.skip(reason="Not implemented")
@pytest.mark.skipif(DOCKER, reason="only works on raspberry pi")
def test_precision_navigation():
    pass


@pytest.mark.skip(reason="Not implemented")
@pytest.mark.skipif(DOCKER, reason="only works on raspberry pi")
def test_search():
    pass


@pytest.mark.skip(reason="Not implemented")
@pytest.mark.skipif(DOCKER, reason="only works on raspberry pi")
def test_station_keeping():
    pass