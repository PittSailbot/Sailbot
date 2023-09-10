"""
calibrates and default values for Odrive and handles interfacing between Odrive and python code
"""

import sys
import threading
import traceback
from time import sleep

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sailbot.constants as c
from sailbot.utils.utils import dummyObject


class Odrive:
    def __init__(self, parent, calibrate=False):
        self._node = parent
        self.logging = self._node.get_logger()
        self.logging.debug("virtual Odrive Starting")

        self.od = self

        self.axis0 = dummyObject()
        self.axis0.requested_state = 8
        self.axis0.motor = dummyObject()
        self.axis0.motor.config = dummyObject()
        self.axis0.motor.current_control = dummyObject()
        self.axis0.motor.current_control.Iq_setpoint = 0
        self.axis0.encoder = dummyObject()
        self.axis0.encoder.config = dummyObject()
        self.axis0.encoder.pos_estimate = 0
        self.axis0.encoder.vel_estimate = 0
        self.axis0.controller = dummyObject()
        self.axis0.controller.config = dummyObject()
        self.axis0.controller.input_pos = 0

        self.axis1 = dummyObject()
        self.axis1.requested_state = 8
        self.axis1.motor = dummyObject()
        self.axis1.motor.config = dummyObject()
        self.axis1.motor.current_control = dummyObject()
        self.axis1.motor.current_control.Iq_setpoint = 1
        self.axis1.encoder = dummyObject()
        self.axis1.encoder.config = dummyObject()
        self.axis1.encoder.pos_estimate = 1
        self.axis1.encoder.vel_estimate = 1
        self.axis1.controller = dummyObject()
        self.axis1.controller.config = dummyObject()
        self.axis1.controller.input_pos = 1

        self.setConstants()

        self.pub = self._node.create_publisher(String, "odriveStatus", 10)
        timer_period = 0.5  # seconds
        self.timer = self._node.create_timer(timer_period, self.publishTimer_callback)

    def publishTimer_callback(self):
        msg = String()
        stringData = ""
        stringData += f"{self.axis0.requested_state},"
        stringData += f"{self.axis0.encoder.pos_estimate},"
        stringData += f"{self.axis0.controller.input_pos},"
        stringData += f"{self.axis0.encoder.vel_estimate},"
        stringData += f"{0} [{self.current0}]:"

        stringData += f"{self.axis1.requested_state},"
        stringData += f"{self.axis1.encoder.pos_estimate},"
        stringData += f"{self.axis1.controller.input_pos},"
        stringData += f"{self.axis1.encoder.vel_estimate},"
        stringData += f"{0} [{self.current1}]"

        msg.data = stringData
        self.pub.publish(msg)
        self.logging.debug('Publishing: "%s"' % msg.data)

        self.axis0.requested_state += 1
        self.axis1.requested_state += 1

    def setConstants(self):
        self.KVRating = c.config["ODRIVE"]["motorKV"]
        # self.od.config.brake_resistance = c.config['CONSTANTS']['odrivebreakresistor']

        self.axis0 = self.od.axis0
        self.mo0 = self.axis0.motor
        self.enc0 = self.axis0.encoder

        self.axis0.controller.config.enable_overspeed_error = False
        self.enc0.config.cpr = c.config["ODRIVE"]["odriveEncoderCPR0"]
        self.od.axis0.motor.config.pole_pairs = c.config["ODRIVE"]["odrivepolepairs0"]
        self.od.axis0.motor.config.torque_constant = (
            1  # read the getting started guide on this, to be changed later
        )
        self.od.axis0.motor.config.motor_type = 0

        self.axis0.controller.config.vel_limit = c.config["ODRIVE"]["velLimit0"]
        self.axis0.controller.config.pos_gain = c.config["ODRIVE"]["posGain0"]
        self.axis0.controller.config.vel_gain = c.config["ODRIVE"]["velGain0"]
        self.axis0.controller.config.vel_integrator_gain = c.config["ODRIVE"][
            "velIntegratorGain0"
        ]
        self.current0 = c.config["ODRIVE"]["currentLimit"]

        self.axis1 = self.od.axis1
        self.mo1 = self.axis1.motor
        self.enc1 = self.axis1.encoder

        self.axis1.controller.config.enable_overspeed_error = False
        self.enc1.config.cpr = c.config["ODRIVE"]["odriveEncoderCPR1"]
        self.od.axis1.motor.config.pole_pairs = c.config["ODRIVE"]["odrivepolepairs1"]
        self.od.axis1.motor.config.torque_constant = (
            1  # read the getting started guide on this, to be changed later
        )
        self.od.axis1.motor.config.motor_type = 0

        self.axis1.controller.config.vel_limit = c.config["ODRIVE"]["velLimit1"]
        self.axis1.controller.config.pos_gain = c.config["ODRIVE"]["posGain1"]
        self.axis1.controller.config.vel_gain = c.config["ODRIVE"]["velGain1"]
        self.axis1.controller.config.vel_integrator_gain = c.config["ODRIVE"][
            "velIntegratorGain1"
        ]
        self.current1 = c.config["ODRIVE"]["currentLimit"]

    @property
    def pos(self):
        return (self.pos0, self.pos1)

    @pos.setter
    def pos(self, value):
        # sets both motor's position to value
        self.pos0 = value
        self.pos1 = value

    def posSet(self, axis, value):
        # sets 'axis' motor to value
        # 'axis' is axis0 or axis1 object
        # self.logging.debug(F"odrive posSet {value}")
        if axis == self.axis0:
            try:
                self.axis0.controller.input_pos = value
            except Exception as e:
                self.logging.error(f"Error setting axis0 to {value}")
        elif axis == self.axis1:
            try:
                self.axis1.controller.input_pos = value
            except Exception as e:
                self.logging.error(f"Error setting axis1 to {value}")

    @property
    def vel(self):
        return (self.vel0, self.vel1)

    @vel.setter
    def vel(self, value):
        self.vel0 = value
        self.vel1 = value

    def velSet(self, axis, value):
        if axis == self.axis0:
            self.vel0 = value
        elif axis == self.axis1:
            self.vel1 = value

    @property
    def torque(self):
        # this will change current drawn
        return 8.27 * getDrawnCurrent / self.KVRating

    def torqueSet(self, axis, value):
        if axis == self.axis0:
            self.torque0 = value
        elif axis == self.axis1:
            self.torque1 = value

    @property
    def current(self):
        return (self.current0, self.current1)

    @current.setter
    def current(self, value):
        self.current0 = value
        self.current1 = value

    def currentSet(self, axis, value):
        # this will change torque!!!
        if axis == self.axis0:
            self.current0 = value
        elif axis == self.axis1:
            self.current1 = value

    def getDemandedCurrent(self):
        return (self.getDemandedCurrent0(), self.getDemandedCurrent1())

    @property
    def pos0(self):
        return self.axis0.encoder.pos_estimate

    @pos0.setter
    def pos0(self, value):
        self.axis0.controller.input_pos = value

    @property
    def vel0(self):
        return self.axis0.controller.config.vel_limit
        # return self.axis0.controller.input_vel

    @vel0.setter
    def vel0(self, value):
        self.axis0.controller.config.vel_limit = value

    @property
    def torque0(self):
        # this will change current drawn
        return self.axis0.controller.input_torque

    @torque0.setter
    def torque0(self, value):
        self.axis0.controller.input_torque = value

    @property
    def current0(self):
        return self.mo0.config.current_lim

    @current0.setter
    def current0(self, value):
        # this will change torque!!!
        # self.torque = (8.27 * value / self.KVRAting)
        # self.logging.warning(F"Warning: Changing the current limit will affect the torque")
        if float(value) > 65:
            raise Exception(
                "Motor current limit should not be raised this high without verifying the motor can handle it"
            )
        self.mo0.config.current_lim = value

    def getDemandedCurrent0(self):
        return self.axis0.motor.current_control.Iq_setpoint

    @property
    def pos1(self):
        return self.axis1.encoder.pos_estimate

    @pos1.setter
    def pos1(self, value):
        self.axis1.controller.input_pos = value

    @property
    def vel1(self):
        return self.axis0.controller.config.vel_limit

    @vel1.setter
    def vel1(self, value):
        self.axis1.controller.config.vel_limit = value

    @property
    def torque1(self):
        # this will change current drawn
        return 8.27 * getDrawnCurrent / self.KVRating

    @torque1.setter
    def torque1(self, value):
        self.axis1.controller.input_torque = value

    @property
    def current1(self):
        return self.mo1.config.current_lim

    @current1.setter
    def current1(self, value):
        # this will change torque!!!
        # self.torque = (8.27 * value / self.KVRAting)
        # print(F"Warning: Changing the current limit will affect the torque")
        if float(value) > 65:
            raise Exception(
                "Motor current limit should not be raised this high without verifying the motor can handle it"
            )
        self.mo1.config.current_lim = value

    def getDemandedCurrent1(self):
        return self.axis1.motor.current_control.Iq_setpoint


def printCurrent(drv):
    while True:
        print(abs(drv.getDemandedCurrent0()))
        sleep(0.5)
