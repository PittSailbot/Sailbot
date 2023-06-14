import os
from rclpy.node import Node


def singleton(cls):
    """A decorator which prevents multiple instances of the same class from being created.
    Useful for physical or __init__ heavy objects where we only want one instance to exist.
        - Import, then use @singleton before class definition"""
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
