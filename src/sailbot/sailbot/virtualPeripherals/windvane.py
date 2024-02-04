"""
reads value from I2C rotery encoder sensor
"""

import sailbot.constants as c


class WindVane:
    def __init__(self):
        pass

    @property
    def angle(self):
        return 0

    @property
    def position(self):
        return 0

    @property
    def noGoMin(self):
        return 360 - int(c.config["MAIN"]["no_go_angle"]) / 2

    @property
    def noGoMax(self):
        return int(c.config["MAIN"]["no_go_angle"]) / 2


def main():
    wv = windVane()
    while True:
        sleep(0.1)
        print(f"Angle {wv.position}")


if __name__ == "__main__":
    main()
