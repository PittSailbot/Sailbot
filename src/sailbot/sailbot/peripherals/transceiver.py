"""
reads and sends data from the connected USB transceiver
"""
# from messages_pb2 import *
import serial
import smbus2 as smbus  # ,smbus2
import time
import logging

from src.sailbot.sailbot import constants as c

I2C_SLAVE_ADDRESS = 0x10


class Transceiver:
    def __init__(self, port_num):
        # connect to device on 'port_num'
        self.ser1 = serial.Serial(port_num, int(c.config["MAIN"]["baudrate"]), timeout=0.5)
        self.I2Cbus = smbus.SMBus(1)

        self.gps = None
        self.rudder_angle = None
        self.sail_angle = None
        self.compass_angle = None
        self.wind_speed = None
        self.wind_direction = None
        self.battery = None

    def send(self, data):
        if type(data) is not str:
            data = str(data)
        self.ser1.write(str(data).encode())

    def send_heartbeat(self):
        """Sends basic boat information in the format:
        [latitude, longitude, rudder_angle, sail_angle, compass_angle, wind_speed, wind_direction, battery]
        """
        data_str = (
            f"DATA: {self.gps.latitude},{self.gps.latitude},{self.rudder_angle},{self.sail_angle},"
            f"{self.compass_angle},{self.wind_speed},{self.wind_direction},{self.battery}"
        )
        self.send(data_str)

    def read(self):
        """Reads incoming data from shore"""
        message = self.ser1.readline().replace("\r\n'", "").replace("b'", "").replace("\\r\\n'", "")

        if message is None or message is "":
            return

        logging.info(f"Received message {message}")
        self.send("Received message")
        # splits = msg.split(" ")
        # return [F"{splits[0]} {splits[1]}", F"{splits[2]} {splits[3]}"]  # format data
        return str(message)

    def readMessages(self, msgOR=None):
        """
        Read messages from transceiver
        """
        if msgOR != None:
            msgs = msgOR
        else:
            # msgs = self.arduino.read()[:-3].replace('\n', '')
            msgs = self.arduino.readData()
        # print(msgs)

        try:
            for msg in msgs:
                processed = False  # to track if we processed the message, if not we can handle an invalid message
                ary = msg.split(" ")
                # make it so you cant do manual commands unless in RC mode to avoid automation undoing work done
                # override is in place to switch to RC if given RC commands if potential accidents may happen
                if len(ary) > 0:
                    # manual adjust sail
                    if ary[0] == "sail" or ary[0] == "S":
                        if self.override:
                            logging.info("OVERRIDE: Switching to RC")
                            self.MODE_SETTING = c.config["MODES"]["MOD_RC"]
                            self.manualControl = True

                        if self.manualControl:
                            logging.info(f"Received message to adjust sail to {float(ary[1])}")
                            self.targetSail = float(ary[1])
                            processed = True
                        else:
                            logging.info("Refuse to change sail, not in RC Mode")

                        # manual adjust rudder
                    elif ary[0] == "rudder" or ary[0] == "R":
                        if self.override:
                            logging.info("OVERRIDE: Switching to RC")
                            self.MODE_SETTING = c.config["MODES"]["MOD_RC"]
                            self.manualControl = True

                        if self.manualControl:
                            rudderMidPoint = (
                                float(c.config["CONSTANTS"]["rudder_angle_max"])
                                - float(c.config["CONSTANTS"]["rudder_angle_min"])
                            ) / 2
                            halfDeadZone = float(c.config["CONSTANTS"]["ControllerRudderDeadZoneDegs"]) / 2

                            # ignore values within the dead zone, ex: if controller is at 1 degree, rudder will still default to 0
                            if (
                                float(ary[1]) < rudderMidPoint + halfDeadZone
                                and float(ary[1]) > rudderMidPoint - halfDeadZone
                            ):
                                self.targetRudder = (
                                    float(rudderMidPoint) - 45
                                )  # values read in range from 0:90 instead of -45:45
                            else:
                                self.targetRudder = (
                                    float(ary[1]) - 45
                                )  # values read in range from 0:90 instead of -45:45
                            logging.info(f"Received message to adjust rudder to {float(ary[1])}")
                            processed = True
                        else:
                            logging.info("Refuse to change sail, not in RC Mode")

                    elif ary[0] == "override":
                        self.override = not self.override
                        processed = True

                    elif (
                        ary[0] == "mode"
                    ):  # set boats mode to the value specified, command should be formatted : "mode {value}" value is int 1-5
                        try:
                            if int(ary[1]) < 0 or int(ary[1]) > 5:
                                logging.info("Outside mode range")
                            else:
                                logging.info(f"Setting mode to {int(ary[1])}")
                                self.MODE_SETTING = int(ary[1])

                                logging.info(f"Setting event array")
                                self.event_arr = []
                                for i in range(len(ary) - 2):
                                    self.event_arr.append(ary[i + 2])

                                processed = True
                        except Exception as e:
                            print(f"Error changing mode: {e}")
                            logging.info(f"Error changing mode: {e}")

                        if self.MODE_SETTING == c.config["MODES"]["MOD_RC"]:
                            logging.info("Setting manual mode to True")
                            self.manualControl = True
                        else:
                            logging.info("Setting manual mode to False")
                            self.manualControl = False

                    elif ary[0] == "addTarget":  # add current GPS to list of targets
                        while self.gps.latitude == None or self.gps.longitude == None:
                            print("no gps")
                            # self.gps.updategps()
                            sleep(0.1)
                        target = (self.gps.latitude, self.gps.longitude)
                        logging.info(f"added Target at {target}")
                        self.targets.append(target)
                        print(f"added target, current target list is {self.targets}")
                        processed = True
                    elif ary[0] != "":
                        print(f"unknown command {ary[0]}")

                if processed:
                    pass
                    # self.arduino.send("boat probably processed message")

        except Exception as e:
            logging.info(f"failed to read command {msgs}")
            # print(f"message error: {e}")
            x = [letter for letter in msgs]
            # print(ary, msgs, x)


def ConvertStringsToBytes(src):
    converted = []
    for b in src:
        converted.append(ord(b))
    return converted


if __name__ == "__main__":
    print("start")
    try:
        ardu = arduino(c.config["MAIN"]["ardu_port"])
        if ardu.readData() == "'":
            # print("error:", ardu.readData())
            raise Exception("could not read arduino data")
    except:
        ardu = arduino(c.config["MAIN"]["ardu_port2"])
        if ardu.readData() == "'":
            # print("error:", ardu.readData())
            raise Exception("could not read arduino data")

    time.sleep(1)
    print("start2")
    while True:
        print(ardu.readData())
