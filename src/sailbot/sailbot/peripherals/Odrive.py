"""
Interface for the rudder and sail ODrive motors.
Run as a script to calibrate the presets. Running 'odrivetool' in cmd is also helpful for debugging.

Documentation: https://docs.odriverobotics.com/v/latest/index.html
"""
import sys
import traceback
from time import sleep
import time

import odrive
from odrive.enums import *
import odrive.utils as ut
from rclpy.node import Node
import rclpy
from std_msgs.msg import String
import threading

from sailbot import constants as c


class Odrive:
    """
    Controls the Odrive motors which move the sails and rudder
    Attributes:
        - pos (float): the position of the motor shaft (in rotations: 1 = 360 deg)
        - vel (float?): the speed at which the motor shaft rotates
        - torque (float?): The force applied to the motor shaft
    Functions:
        - reboot(): reinitializes the odrive
        - calibrate(): used to tune the odrive's settings
    """

    od = None

    def __init__(self, preset, calibrate=False):
        """
        Args:
            preset (str): pre-defined odrive configurations
                - Supports either 'sail' or 'rudder'
            calibrate (bool): whether to start the odrive in calibration mode
        """
        self.axis = None
        self._node = Node(f"odrive_{preset}")
        self.logging = self._node.get_logger()
        self.logging.debug(f"Initializing ODrive with preset: {preset}")
        self.preset = preset

        if self.od == None:
            self.od = odrive.find_any()
            self.last_connect_time = time.time()
            self.od.clear_errors()

        self.axis = self.od.axis0 if self.preset == c.config["ODRIVE"]["m0"] else self.od.axis1

        if calibrate:
            self.calibrate()

        self.offset = 0

        self.axis.encoder.config.cpr = c.config["ODRIVE"]["odriveEncoderCPR"]
        self.axis.motor.config.pole_pairs = c.config["ODRIVE"]["odrivepolepairs"]

        self.axis.motor.config.torque_constant = 1  # read the getting started guide on this, to be changed later
        self.axis.motor.config.motor_type = 0

        self.KVRating = int(c.config["ODRIVE"]["motorKV"])
        self.current_limit = int(c.config["ODRIVE"]["currentLimit"])
        self.axis.motor.config.current_lim = 20  # [A] current lim not being set to motor?

        self.axis.controller.config.enable_overspeed_error = False
        self.od.config.brake_resistance = float(c.config["ODRIVE"]["odrivebreakresistor"])

        if preset == "sail":
            self.axis.controller.config.pos_gain = int(c.config["ODRIVE_SAIL"]["posGain"])

            self.axis.controller.config.vel_gain = float(c.config["ODRIVE_SAIL"]["velGain"])
            self.axis.controller.config.vel_limit = float(c.config["ODRIVE_SAIL"]["velLimit"])
            self.axis.controller.config.vel_integrator_gain = int(c.config["ODRIVE_SAIL"]["velIntegratorGain"])

            self.max_rotations = float(c.config["ODRIVE_SAIL"]["max_rotations"])

        elif preset == "rudder":
            self.axis.controller.config.pos_gain = int(c.config["ODRIVE_RUDDER"]["posGain"])

            self.axis.controller.config.vel_gain = float(c.config["ODRIVE_RUDDER"]["velGain"])
            self.axis.controller.config.vel_limit = float(c.config["ODRIVE_RUDDER"]["velLimit"])
            self.axis.controller.config.vel_integrator_gain = int(c.config["ODRIVE_RUDDER"]["velIntegratorGain"])

            self.max_rotations = float(c.config["ODRIVE_RUDDER"]["max_rotations"])

        else:
            raise ValueError(f"Trying to load an undefined preset: {preset}")

        self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        ut.dump_errors(self.od)
        sleep(0.1)

    def cleanup():
        if Odrive.od:
            od = Odrive.od
        else:
            od = odrive.find_any()

        od.axis0.requested_state = AXIS_STATE_IDLE
        od.axis1.requested_state = AXIS_STATE_IDLE

    def reboot(self):
        """Reboots both sail and rudder"""
        try:
            self.od.reboot()
        except Exception:
            # error is expected
            pass
        sleep(2)
        self.__init__(preset=self.preset)

    def calibrate(self):
        self.logging.warning("Calibrating")

        self.axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        count = 0
        sleep(5)
        while self.axis.current_state != AXIS_STATE_IDLE:
            sleep(0.5)
            count += 1
            if count > 40:
                self.logging.warning("Odrive is taking a long time to calibrate the motors")
                count = 0
        ut.dump_errors(self.od)
        sleep(1)

    def reconnect(self, preset):
        if time.time() - self.last_connect_time > 1:
            self.logging.warning("Odrive Reconnecting")
            self.od = odrive.find_any()
            self.last_connect_time = time.time()
            self.preset = preset
            self.axis = self.od.axis0 if self.preset == c.config["ODRIVE"]["m0"] else self.od.axis1

    @property
    def pos(self):
        return self.axis.encoder.pos_estimate

    @pos.setter
    def pos(self, value):
        if value < -self.max_rotations or value > self.max_rotations:
            raise ValueError(f"Tried setting odrive position above max limit {value}")

        self.axis.controller.input_pos = value + self.offset

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
        return self.axis.motor.config.current_lim

    @current_limit.setter
    def current_limit(self, value):
        # this will change torque!!!
        # self.torque = (8.27 * value / self.KVRAting)
        if value > 65:
            raise ValueError("Motor current limit should not be raised this high without verifying the motor can handle it")
        else:
            self.axis.motor.config.current_lim = value

    def getDemandedCurrent(self):
        return self.axis.motor.current_control.Iq_setpoint


if __name__ == "__main__":
    print("Run as sudo if on rasp pi, will otherwise not work")
    rclpy.init()

    rudder = Odrive(preset="rudder", calibrate=False)
    sail = Odrive(preset="sail", calibrate=False)

    last_rudder_move_val = 0
    motor = None

    while True:
        try:
            string = input("  > Enter Input: ").lower()
            args = string.split(" ")

            if args[0] == "r" or args[0] == "rudder":
                motor = rudder
            elif args[0] == "s" or args[0] == "sail":
                motor = sail
            elif args[0] == "calibrate":
                rudder.calibrate()
                sail.calibrate()
            else:
                raise ValueError(f"Invalid argument for motor, expected rudder or sail, got {args[0]}")

            if args[1] == "pos_gain":
                # Set pos_gain
                if len(args) == 2:
                    print(motor.axis.controller.config.pos_gain)
                else:
                    val = float(args[2])
                    print(motor.axis.controller.config.pos_gain, "->", val)
                    motor.axis.controller.config.pos_gain = val

            elif args[1] == "offset":
                # Set pos + offset
                if len(args) == 2:
                    print(motor.pos + motor.offset)
                else:
                    val = float(args[2])
                    print(motor.pos + motor.offset, "->", val)
                    motor.pos = val + motor.offset

            elif args[1] == "offset_reset":
                # Set pos + offset, then reset to 0
                if len(args) == 2:
                    print(motor.pos + motor.offset)
                else:
                    val = float(args[2])
                    print(motor.pos + motor.offset, "->", val, "(resetting to 0 in 1 sec)")
                    motor.pos = val + motor.offset
                    last_rudder_move_val = val

                    def rudderReset(*args):
                        sleep(1)
                        motor.pos = motor.offset

                    threading.Thread(target=rudderReset).start()

            elif args[1] == "z":
                # Move to last set pos
                print(motor.pos + motor.offset, "->", last_rudder_move_val, "(resetting to 0 in 1 sec)")
                motor.pos = last_rudder_move_val + motor.offset

                def rudderReset(*args):
                    sleep(1)
                    motor.pos0 = motor.offset

                threading.Thread(target=rudderReset).start()

            elif args[1] == "or":
                # idk
                # val = float(string.split(' ')[1])
                motor.offset = motor.pos + motor.offset
                print("offset is", motor.offset)

            elif args[1] == "os":
                # idk
                # val = float(string.split(' ')[1])
                motor.offset = motor.pos + motor.offset
                print("offset is", motor.offset)

            elif args[1] == "velocity":
                if len(args) == 2:
                    print(motor.vel)
                else:
                    val = float(string[1:])
                    print(motor.vel, "->", val)
                    motor.vel = val

            elif args[1] == "current":
                if len(args) == 2:
                    print(f"{motor.getDemandedCurrent()} [{motor.current}]")
                else:
                    val = float(string[1:])
                    print(motor.current, "->", val)
                    motor.current = val

            elif args[1] == "vel_int_gain":
                if len(args) == 2:
                    print(motor.axis.controller.config.vel_integrator_gain)
                else:
                    val = float(string.split(" ")[1])
                    print(motor.axis.controller.config.vel_integrator_gain, "->", val)
                    motor.axis.controller.config.vel_integrator_gain = val

            elif args[1] == "vel_gain":
                if len(args) == 2:
                    print(motor.axis.controller.config.vel_gain)
                else:
                    val = float(args[3])
                    print(motor.axis.controller.config.vel_gain, "->", val)
                    motor.axis.controller.config.vel_gain = val

            elif args[1] == "reset":
                print("rebooting")
                motor.reboot()
                motor = Odrive(calibrate=False)

            elif args[1] == "e":
                ut.dump_errors(motor.od)

            else:
                val = float(args[1])
                motor.pos = val
        except KeyboardInterrupt as e:
            ut.dump_errors(motor.od)
            break
        except Exception as e:
            print(f"Error: {e}")
            print(traceback.format_exc())

    print("done")
