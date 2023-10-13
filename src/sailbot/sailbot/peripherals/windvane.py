"""
Interface for reading wind angle
"""
from threading import Lock, Thread
from time import sleep

import board
from adafruit_seesaw import digitalio, rotaryio, seesaw
from RPi import GPIO
from rclpy.node import Node
from std_msgs.msg import String

from sailbot import constants as c


class WindVane(Node):
    """Measures the angle of the wind

    Attributes:
        angle (float): the angle pointing into the wind
        no_go_zone (NoGoZone): the left and right limits of the no go zone
    """

    def __init__(self):
        self.stepsPerRev = 256

        self._position = 0

        # pin numbers
        self.clk = 17
        self.dt = 18
        self.hef = 23

        # self.q = Queue(0)

        # self.saw = seesaw.Seesaw (board.I2C(), 0x36)
        # self.encoder = rotaryio.IncrementalEncoder(self.saw)
        self.lock = Lock()
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.clk, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.dt, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.hef, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.offset = 0
        self.counter = 0
        self.clkLastState = GPIO.input(self.clk)
        # pump_thread = Thread(target=self.run)
        # pump_thread.start()

        # pump_thread2 = Thread(target=self.flush_queue)
        # pump_thread2.start()

        pump_thread3 = Thread(target=self.update)
        pump_thread3.start()

        self.no_go_zone = NoGoZone(wind_direction=self.angle)

    def map(self, x, min1, max1, min2, max2):
        # converts value x, which ranges from min1-max1, to a corresponding value ranging from min2-max2
        # ex: map(0.3, 0, 1, 0, 100) returns 30
        # ex: map(70, 0, 100, 0, 1) returns .7
        x = min(max(x, min1), max1)
        return min2 + (max2 - min2) * ((x - min1) / (max1 - min1))

    @property
    def angle(self):
        counter = self.position
        while counter < 0:
            counter += self.stepsPerRev

        counter = counter % self.stepsPerRev
        return self.map(counter, 0, self.stepsPerRev - 1, 0, 359)

    @property
    def position(self):
        self.lock.acquire()
        # self.checkHef()
        # val =  int( ((self.encoder.position * (360/self.stepsPerRev)) - self.offset)%360 )
        val = int(((self._position * (360 / self.stepsPerRev)) - self.offset) % 360)
        self.lock.release()
        return val

    def flush_queue(self):
        while True:
            self.counter += self.q.get()

    def checkHef(self):
        hefState = GPIO.input(self.hef)
        if hefState == False:
            self.offset = int((self._position * 1.8) % 360)
            return False
        return True

    def update(self):
        while True:
            clkState = GPIO.input(self.clk)
            dtState = GPIO.input(self.dt)
            hefState = GPIO.input(self.hef)
            if clkState != self.clkLastState:
                if dtState == 1 and clkState == 0:
                    self._position += 1

                elif dtState == 1 and clkState == 1:
                    self._position -= 1

                self.clkLastState = clkState

            # print(dtState, clkState)

            # if hefState == False:
            #     self.counter = 0


class NoGoZone:
    """Simplifies checking whether a compass heading is inside the no go zone
    - Declare a WindVane object and write 'if x in windvane.no_go_zone'

    Attributes:
        left_bound (float): the compass angle of the left-most no go zone bound
        right_bound (float): the compass angle of the right-most no go zone bound
    """

    NO_GO_RANGE = int(c.config["WINDVANE"]["no_go_range"]) / 2

    def __init__(self, wind_direction):
        self.left_bound = (wind_direction - self.NO_GO_RANGE) % 360
        self.right_bound = (wind_direction + self.NO_GO_RANGE) % 360

    def __contains__(self, heading):
        if self.left_bound > self.right_bound:
            heading = heading % 360

            # Check if the heading is within the wrapped bounds
            if heading >= self.left_bound or heading <= self.right_bound:
                return True
        else:
            # Bounds don't wrap around
            if self.left_bound <= heading <= self.right_bound:
                return True

        return False


def main():
    wv = WindVane()
    while True:
        sleep(0.1)
        print(f"Angle {wv.position}")


if __name__ == "__main__":
    main()
