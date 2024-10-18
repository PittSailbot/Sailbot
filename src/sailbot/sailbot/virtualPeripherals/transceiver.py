"""
reads and sends data from the connected USB transceiver
"""

I2C_SLAVE_ADDRESS = 0x10


def ConvertStringsToBytes(src):
    converted = []
    for b in src:
        converted.append(ord(b))
    return converted


class Transceiver:
    def __init__(self):
        pass

    def send(self, data):
        print(data)
        pass

    def readData(self):
        # get data from transceiver
        return ""

    def read(self):
        return ""
