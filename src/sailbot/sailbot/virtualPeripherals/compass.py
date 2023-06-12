"""
Handles interfacing with the I2C compass and accelerometer sensor
"""

import sailbot.boatMath

class compass():
    def __init__(self):
        pass

    @property
    def vector(self):
        return (0,0,0) # (mag_x, mag_y, mag_z)

    @property
    def angle_X(self):
        # return X component of compass, occasionally the compass fails to read, if this happens 10 times in a row raise error
        return 0
    @property
    def angle_Y(self):
        # return Y component of compass, occasionally the compass fails to read, if this happens 10 times in a row raise error
        return 0
    @property
    def angle_Z(self):
        # return Z component of compass, occasionally the compass fails to read, if this happens 10 times in a row raise error
        return 0
    
    @property
    def angleToNorth(self):
        return 0
    
    @property
    def angle(self):
        # returns smoothed angle measurement
        return 0

if __name__ == "__main__":
    comp = compass()
    while True:
        print(comp.angle)
        sleep(.2)
        

    


