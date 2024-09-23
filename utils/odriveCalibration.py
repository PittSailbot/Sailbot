import odrive
from odrive.enums import *
import time
import odrive.utils as ut
import sys

def find_odrive():
    print("Finding an ODrive...")
    odrv0 = odrive.find_any()
    print("ODrive found.")
    return odrv0

def calibrate(axes):
    for axis in axes:
        axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    # Wait for calibration to finish
    for axis in axes:
        while axis.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)


def main():
    odrv0 = find_odrive()
    odrv0.clear_errors()

    axes = [odrv0.axis0, odrv0.axis1]

    if sys.argv == '0':
        axes = [odrv0.axis0]
    elif sys.argv == '1':
        axes = [odrv0.axis1]

    print("Calibrating...")
    calibrate(axes)

    print("Calibration complete.")
    ut.dump_errors(odrv0)

if __name__ == "__main__":
    main()
