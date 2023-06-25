import os
from rclpy.node import Node
from rclpy.executors import ShutdownException, TimeoutException
import rclpy
from datetime import datetime


def singleton(cls):
    """A decorator which prevents duplicate classes from being created.
    Useful for physical objects where only one exists.
        - Import, then invoke use @singleton before class definition"""
    instances = {}

    def get_instance(*args, **kwargs):
        if cls not in instances:
            instances[cls] = cls(*args, **kwargs)
        return instances[cls]

    return get_instance


def map(self, x, min1, max1, min2, max2, enforce_limits=False):
    # converts value x, which ranges from min1-max1, to a corresponding value ranging from min2-max2
    # ex: map(0.3, 0, 1, 0, 100) returns 30
    # ex: map(70, 0, 100, 0, 1) returns .7
    if enforce_limits:
        x = min(max(x, min1), max1)
    return min2 + (max2 - min2) * ((x - min1) / (max1 - min1))


class dummyObject:
    """
    a class that can be used as a placeholder for something else, usually a physical sensor that is not present.
    allows for setting of arbitrary variables that can be read and written to
    """

    def __init__(self, *args, **kwargs):
        pass


def ros_spin_some(node, executor=None, timeout_sec=0, wait_condition=lambda: False):
    """
    execute ros callbacks until there are no more available or for timeout_sec, whichever comes first
    if timeout_sec is 0 then it will execute callbacks until there are no more available
    """
    if timeout_sec != 0:
        endTime = datetime.now() + timeout_sec
    executor = rclpy.get_global_executor() if executor is None else executor
    executor.add_node(node)
    while True:
        if timeout_sec != 0:
            remainingTime = endTime - datetime.now()
            if remainingTime <= 0:
                break
        else:
            remainingTime = 0.0

        try:
            handler, _, _ = executor.wait_for_ready_callbacks(
                remainingTime, None, wait_condition
            )
        except ShutdownException:
            pass
        except TimeoutException:
            break
        else:
            handler()
            if handler.exception() is not None:
                raise handler.exception()

    executor.remove_node(node)
