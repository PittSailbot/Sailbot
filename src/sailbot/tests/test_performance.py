"""
Benchmark runtime performance across versions
"""
import os
import datetime
import pytest
import cProfile
from pstats import SortKey, Stats

from sailbot import main
from sailbot import constants as c

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if DOCKER == "True" else False

if not DOCKER:
    import rclpy

    rclpy.init()


@pytest.mark.skipif(DOCKER, reason="only works on raspberry pi")
def test_main():
    boat = main.Boat()

    with cProfile.Profile() as profile:
        for i in range(0, 10):
            boat.main_loop()

        Stats(profile).strip_dirs().sort_stats(SortKey.CUMULATIVE).print_stats()

