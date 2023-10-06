"""
calibrates and default values for Odrive and handles interfacing between Odrive and python code
"""
import sys
import traceback
from time import sleep

import odrive
import odrive.utils as ut
from rclpy.node import Node
from std_msgs.msg import String

from sailbot import constants as c


class Odrive:
    """
    Controls the Odrive motors which move the sails and rudder
    Attributes:
        - pos (float?): the position of the motor shaft
        - vel (float?): the speed at which the motor shaft rotates
        - torque (float?): The force applied to the motor shaft
    Functions:
        - reboot(): reinitializes the odrive
        - calibrate(): used to tune the odrive's settings
    """

    def __init__(self, preset=None, calibrate=False):
        """
        Args:
            preset (str): pre-defined odrive configurations
                - Supports either 'sail' or 'rudder'
            calibrate (bool): whether to start the odrive in calibration mode
        """
        self._node = Node("odrive")
        self.logging = self._node.get_logger()
        self.logging.debug(f"Initializing ODrive with preset: {preset}")

        self.od = odrive.find_any()
        self.preset = preset

        # Switch if sail and rudder are reversed
        if self.preset == "sail":
            self.axis = self.od.axis0
        elif self.preset == "rudder":
            self.axis = self.od.axis1

        self.mo = self.axis.motor
        self.enc = self.axis.encoder

        self.offset = 0

        self.enc.config.cpr = c.config["ODRIVE"]["odriveEncoderCPR"]
        self.axis.motor.config.pole_pairs = c.config["ODRIVE"]["odrivepolepairs"]

        self.axis.motor.config.torque_constant = 1  # read the getting started guide on this, to be changed later
        self.axis.motor.config.motor_type = 0

        self.KVRating = int(c.config["ODRIVE"]["motorKV"])
        self.current_limit = int(c.config["ODRIVE"]["currentLimit"])

        self.axis.controller.config.enable_overspeed_error = False
        self.od.config.brake_resistance = float(c.config["ODRIVE"]["odrivebreakresistor"])

        if preset == "sail":
            self.axis.controller.config.pos_gain = int(c.config["ODRIVE_SAIL"]["posGain"])

            self.axis.controller.config.vel_gain = float(c.config["ODRIVE_SAIL"]["velGain"])
            self.axis.controller.config.vel_limit = float(c.config["ODRIVE_SAIL"]["velLimit"])
            self.axis.controller.config.vel_integrator_gain = int(c.config["ODRIVE_SAIL"]["velIntegratorGain"])

            self.rotations_per_degree = float(c.config["ODRIVE_SAIL"]["rotations_per_degree"])

        elif preset == "rudder":
            self.axis.controller.config.pos_gain = int(c.config["ODRIVE_RUDDER"]["posGain"])

            self.axis.controller.config.vel_gain = float(c.config["ODRIVE_RUDDER"]["velGain"])
            self.axis.controller.config.vel_limit = float(c.config["ODRIVE_RUDDER"]["velLimit"])
            self.axis.controller.config.vel_integrator_gain = int(c.config["ODRIVE_RUDDER"]["velIntegratorGain"])

            self.rotations_per_degree = float(c.config["ODRIVE_RUDDER"]["rotations_per_degree"])

        elif preset is not None:
            raise ValueError("Trying to load an undefined preset!")

        else:
            # Default fallback values when no preset is defined
            raise NotImplementedError("Non-preset fallback values haven't been coded yet")

        if calibrate:
            self.calibrate()

        self.axis.requested_state = 8
        ut.dump_errors(self.od)
        sleep(0.1)

    def reboot(self):
        try:
            self.od.reboot()
        except Exception:
            # error is expected
            pass
        sleep(2)
        self.__init__(preset=self.preset)

    def calibrate(self):
        self.logging.info("Calibrating")
        self.reboot()
        self.axis.requested_state = 3
        sleep(15)
        ut.dump_errors(self.od)

    @property
    def pos(self):
        return self.axis.encoder.pos_estimate

    @pos.setter
    def pos(self, value):
        self.axis.controller.input_pos = value

    @property
    def vel(self):
        return self.axis.controller.config.vel_limit
        # return self.axis0.controller.input_vel

    @vel.setter
    def vel(self, value):
        self.axis.controller.config.vel_limit = value

    @property
    def torque(self):
        # this will change current drawn
        # TODO: (TOM) which return statement?
        # getDrawnCurrent typo? or different from getDemandedCurrent()?
        # return 8.27 * getDrawnCurrent / self.KVRating  # Used by torque and torque0
        return self.axis.controller.input_torque  # Used by torque1

    @torque.setter
    def torque(self, value):
        self.axis.controller.input_torque = value

    @property
    def current_limit(self):
        return self.mo.config.current_lim

    @current_limit.setter
    def current_limit(self, value):
        # this will change torque!!!
        # self.torque = (8.27 * value / self.KVRAting)
        if value > 65:
            raise ValueError(
                "Motor current limit should not be raised this high without verifying the motor can handle it"
            )
        else:
            self.mo.config.current_lim = value

    def getDemandedCurrent(self):
        return self.axis.motor.current_control.Iq_setpoint


# TODO: Update to work with new changes
if __name__ == "__main__":
    print("Run as sudo if on rasp pi, will otherwise not work")
    if len(sys.argv) < 2 or sys.argv[1] != "0":
        rudder = Odrive(preset="rudder", calibrate=True)
        sail = Odrive(preset="sail", calibrate=True)
    else:
        rudder = Odrive(preset="rudder", calibrate=False)
        sail = Odrive(preset="sail", calibrate=False)

    while True:
        try:
            string = input("  > Enter Input:")

            if string == "calibrate":
                # print("rebooting")
                # drv.reboot()
                drv = Odrive(calibrate=True)

            elif string.startswith("pi0") or string.startswith("PI0"):
                if len(string.split(" ")) == 1:
                    print(drv.axis0.controller.config.pos_gain)
                else:
                    val = float(string.split(" ")[1])
                    print(drv.axis0.controller.config.pos_gain, "->", val)
                    drv.axis0.controller.config.pos_gain = val

            elif string.startswith("pi1") or string.startswith("PI1"):
                if len(string.split(" ")) == 1:
                    print(drv.axis1.controller.config.pos_gain)
                else:
                    val = float(string.split(" ")[1])
                    print(drv.axis1.controller.config.pos_gain, "->", val)
                    drv.axis1.controller.config.pos_gain = val

            elif string.lower().startswith("p0"):
                if len(string.split(" ")) == 1:
                    print(drv.pos0 + p0_offset)
                else:
                    val = float(string.split(" ")[1])
                    print(drv.pos0 + p0_offset, "->", val)
                    drv.pos0 = val + p0_offset

            elif string.lower().startswith("pr"):
                if len(string.split(" ")) == 1:
                    print(drv.pos0 + p0_offset)
                else:
                    val = float(string.split(" ")[1])
                    print(drv.pos0 + p0_offset, "->", val, "(resetting to 0 in 1 sec)")
                    drv.pos0 = val + p0_offset
                    lastRudderMoveVal = val

                    def rudderReset(*args):
                        sleep(1)
                        drv.pos0 = p0_offset

                    threading.Thread(target=rudderReset).start()

            elif string.lower().startswith("z"):
                print(drv.pos0 + p0_offset, "->", lastRudderMoveVal, "(resetting to 0 in 1 sec)")
                drv.pos0 = lastRudderMoveVal + p0_offset

                def rudderReset(*args):
                    sleep(1)
                    drv.pos0 = p0_offset

                threading.Thread(target=rudderReset).start()

            elif string.lower().startswith("ps"):
                if len(string.split(" ")) == 1:
                    print(drv.pos1 + p1_offset)
                else:
                    val = float(string.split(" ")[1])
                    print(drv.pos1 + p1_offset, "->", val)
                    drv.pos1 = val + p1_offset

            elif string.lower().startswith("or"):
                # val = float(string.split(' ')[1])
                p0_offset = drv.pos0 + p0_offset
                print("offset is", p0_offset)

            elif string.lower().startswith("os"):
                # val = float(string.split(' ')[1])
                p1_offset = drv.pos0 + p0_offset
                print("offset is", p0_offset)

            elif string[0] == "v" or string[0] == "V":
                if len(string) == 1:
                    print(drv.vel)
                else:
                    val = float(string[1:])
                    print(drv.vel, "->", val)
                    drv.vel = val

            elif string[0] == "c" or string[0] == "C":
                if len(string) == 1:
                    print(f"{drv.getDemandedCurrent()} [{drv.current}]")
                else:
                    val = float(string[1:])
                    print(drv.current, "->", val)
                    drv.current = val

            elif string.lower().startswith("d0"):
                if len(string.split(" ")) == 1:
                    print(drv.axis0.controller.config.vel_integrator_gain)
                else:
                    val = float(string.split(" ")[1])
                    print(drv.axis0.controller.config.vel_integrator_gain, "->", val)
                    drv.axis0.controller.config.vel_integrator_gain = val

            elif string.lower().startswith("d1"):
                if len(string.split(" ")) == 1:
                    print(drv.axis1.controller.config.vel_integrator_gain)
                else:
                    val = float(string.split(" ")[1])
                    print(drv.axis1.controller.config.vel_integrator_gain, "->", val)
                    drv.axis1.controller.config.vel_integrator_gain = val

            elif string.lower().startswith("i0"):
                if len(string.split(" ")) == 1:
                    print(drv.axis0.controller.config.vel_gain)
                else:
                    val = float(string.split(" ")[1])
                    print(drv.axis0.controller.config.vel_gain, "->", val)
                    drv.axis0.controller.config.vel_gain = val

            elif string.lower().startswith("i1"):
                if len(string.split(" ")) == 1:
                    print(drv.axis1.controller.config.vel_gain)
                else:
                    val = float(string.split(" ")[1])
                    print(drv.axis1.controller.config.vel_gain, "->", val)
                    drv.axis1.controller.config.vel_gain = val

            elif string == "reset":
                print("rebooting")
                drv.reboot()
                drv = Odrive(calibrate=False)

            elif string[0] == "e" or string[0] == "E":
                ut.dump_errors(drv.od)

            else:
                val = float(string)
                drv.pos0 = val
                drv.pos1 = val
        except KeyboardInterrupt as e:
            ut.dump_errors(drv.od)
            break
        except Exception as e:
            print(f"Error: {e}")
            print(traceback.format_exc())

    print("done")
