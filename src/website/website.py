import os
import threading
import time

import rclpy
from flask import Flask, render_template
from rclpy.node import Node
from std_msgs.msg import String

import sailbot.constants as c
from sailbot.utils import dummyObject

app = Flask(__name__)
app.secret_key = "sailbot"

# dont print unimportant messages
import logging

log = logging.getLogger("werkzeug")
log.setLevel(logging.ERROR)


class Website(Node):
    def __init__(self):
        super().__init__("Website")
        self.logging = self.get_logger()
        self.logging.info("WEBSITE STARTED")
        threading.Thread(target=rclpy.spin, args=[self]).start()
        self.notification = ""
        self.gps = dummyObject()
        self.gps.latitude = -1
        self.gps.longitude = -1
        self.gps.track_angle_deg = -1

        self.compass = dummyObject()
        self.compass.angle = -1

        self.odrive = dummyObject()
        self.odrive.axis0 = dummyObject()
        self.odrive.axis0.requested_state = -1
        self.odrive.axis0.targetPos = -1
        self.odrive.axis0.pos = -1
        self.odrive.axis0.velocity = -1
        self.odrive.axis0.currentDraw = -1

        self.odrive.axis1 = dummyObject()
        self.odrive.axis1.requested_state = -1
        self.odrive.axis1.targetPos = -1
        self.odrive.axis1.pos = -1
        self.odrive.axis1.velocity = -1
        self.odrive.axis1.currentDraw = -1

        self.gps_subscription = self.create_subscription(
            String, "GPS", self.ROS_GPSCallback, 10
        )
        self.compass_subscription = self.create_subscription(
            String, "compass", self.ROS_compassCallback, 10
        )
        self.odrive_subscription = self.create_subscription(
            String, "odriveStatus", self.ROS_odriveCallback, 10
        )

        self.modePub = self.create_publisher(String, "mode", 10)

        self.dataDict = {
            "gps": f"{self.gps.latitude},{self.gps.longitude}",
            "compass": f"{self.compass.angle}",
            "odrive_axis0": f"{self.odrive.axis0.requested_state},{self.odrive.axis0.pos},{self.odrive.axis0.targetPos},{self.odrive.axis0.velocity},{self.odrive.axis0.currentDraw}",
            "odrive_axis1": f"{self.odrive.axis1.requested_state},{self.odrive.axis1.pos},{self.odrive.axis1.targetPos},{self.odrive.axis1.velocity},{self.odrive.axis1.currentDraw}",
        }

        self.waypoints = [
            {"name": "Waypoint 1", "lat": "0.1", "lon": "0"},
            {"name": "Waypoint 2", "lat": "-0.1", "lon": "0"},
        ]

        self.circles = [{"lat": "0", "lon": "-0.1", "radius": 500}]

    def process(self):
        while True:
            rclpy.spin_once()
            sleep(0.05)

    def ROS_GPSCallback(self, string):
        string = string.data
        if string == "None,None,None":
            self.gps.latitude = None
            self.gps.longitude = None
            self.gps.track_angle_deg = None
            return

        lat, long, trackangle = string.replace("(", "").replace(")", "").split(",")
        self.gps.latitude = float(lat)
        self.gps.longitude = float(long)
        self.gps.track_angle_deg = float(trackangle)

        self.dataDict["gps"] = f"{self.gps.latitude},{self.gps.longitude}"

    def ROS_compassCallback(self, string):
        string = string.data
        if string == "None,None":
            self.compass.angle = None
            return

        angle = string.replace("(", "").replace(")", "")
        self.compass.angle = float(angle)

        self.dataDict["compass"] = f"{self.compass.angle}"

    def ROS_odriveCallback(self, string):
        string = string.data
        axis0Data = string.split(":")[0]
        axis1Data = string.split(":")[1]

        for axis, axisData in [
            (self.odrive.axis0, axis0Data),
            (self.odrive.axis1, axis1Data),
        ]:
            axis.requested_state = axisData[0]
            axis.pos = axisData[1]
            axis.targetPos = axisData[2]
            axis.velocity = axisData[3]
            axis.currentDraw = axisData[4]

        self.dataDict[
            "odrive_axis0"
        ] = f"{self.odrive.axis0.requested_state},{self.odrive.axis0.pos},{self.odrive.axis0.targetPos},{self.odrive.axis0.velocity},{self.odrive.axis0.currentDraw}"
        self.dataDict[
            "odrive_axis1"
        ] = f"{self.odrive.axis1.requested_state},{self.odrive.axis1.pos},{self.odrive.axis1.targetPos},{self.odrive.axis1.velocity},{self.odrive.axis1.currentDraw}"

    def publishModeChange(self, modeString):
        msg = String()
        msg.data = f"Set Mode:{modeString}"
        self.modePub.publish(msg)
        self.logging.debug('Publishing: "%s"' % msg.data)


@app.route("/", methods=["GET", "POST"])
def home():
    return render_template("index.html", **DATA.dataDict)


@app.route("/gps")
def gps():
    return f"{DATA.gps.latitude}, {DATA.gps.longitude}"


@app.route("/gpsJSON")
def gpsJSON():
    jsonDict = {"lat": DATA.gps.latitude, "lon": DATA.gps.longitude}
    return jsonDict


@app.route("/waypoints")
def waypoints():
    jsonDict = {"waypoints": DATA.waypoints}
    return jsonDict


@app.route("/circles")
def circles():
    jsonDict = {"circles": DATA.circles}
    return jsonDict


@app.route("/compass")
def compass():
    return f"{DATA.compass.angle}"


@app.route("/axis0")
def axis0():
    return DATA.dataDict["odrive_axis0"]


@app.route("/axis1")
def axis1():
    return DATA.dataDict["odrive_axis1"]


@app.route("/mode/<mode>")
def setMode(mode):
    mappingDict = {
        "manual": c.config["MODES"]["MOD_RC"],
        "avoid": c.config["MODES"]["MOD_COLLISION_AVOID"],
        "nav": c.config["MODES"]["MOD_PRECISION_NAVIGATE"],
        "endurance": c.config["MODES"]["MOD_ENDURANCE"],
        "keeping": c.config["MODES"]["MOD_STATION_KEEPING"],
        "search": c.config["MODES"]["MOD_SEARCH"],
    }

    if mode.lower() in mappingDict:
        DATA.publishModeChange(mappingDict[mode.lower()])
        DATA.notification = f"Mode set: {mode}"

    else:
        DATA.notification = f"ignoring attempt to set mode to unknown value"

    return {}


@app.route("/Notification")
def returnNotification():
    if DATA.notification != "":
        message = DATA.notification

        def resetNotifTimer():
            time.sleep(2)
            if DATA.notification == message:
                DATA.notification = ""

        threading.Thread(target=resetNotifTimer).start()
    return DATA.notification


@app.route("/map")
def root():
    markers = [{"lat": 0, "lon": 0, "popup": "This is the middle of the map."}]
    return render_template("map.html", markers=markers)


def ros_main():
    global DATA, app

    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/website"
    rclpy.init()
    DATA = Website()
    port = int(os.environ.get("PORT", 5000))
    # threading.Thread(target=app.run, kwargs={'debug': True, 'host': '0.0.0.0', 'port': port}).start()

    # threading.Thread(target=DATA.process).start()
    app.run(debug=True, host="0.0.0.0", port=port)
    # rclpy.spin(DATA)


if __name__ == "__main__":
    # main()
    pass
