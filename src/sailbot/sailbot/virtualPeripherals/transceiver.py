"""
reads and sends data from the connected USB transceiver
"""
import sailbot.constants as c
import sailbot.utils as utils
import sys
import time

I2C_SLAVE_ADDRESS = 0x10


def ConvertStringsToBytes(src):
    converted = []
    for b in src:
        converted.append(ord(b))
    return converted


class arduino:
    def __init__(self, port_num):
        # connect to device on 'port_num'
        pass

    def send(self, data):
        # print(data)
        pass

    def readData(self):
        # get data from transceiver
        return ""

    def read(self):
        return ""
