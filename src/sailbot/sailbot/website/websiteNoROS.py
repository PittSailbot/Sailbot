import os
import random
import threading
import time

from flask import Flask, render_template

app = Flask(__name__)


class dummyObject:
    pass


class Website:
    def __init__(self):
        self.gps = dummyObject()
        self.gps.latitude = 42.849135
        self.gps.longitude = -70.986314
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

        self.dataDict = {
            "gps": f"{self.gps.latitude},{self.gps.longitude}",
            "compass": f"{self.compass.angle}",
            "odrive_axis0": f"{self.odrive.axis0.requested_state},{self.odrive.axis0.pos},{self.odrive.axis0.targetPos},{self.odrive.axis0.velocity},{self.odrive.axis0.currentDraw}",
            "odrive_axis1": f"{self.odrive.axis1.requested_state},{self.odrive.axis1.pos},{self.odrive.axis1.targetPos},{self.odrive.axis1.velocity},{self.odrive.axis1.currentDraw}",
        }


@app.route("/")
def home():
    return render_template("index.html", **DATA.dataDict)


@app.route("/gps")
def gps():
    return f"{DATA.gps.latitude}, {DATA.gps.longitude}"


@app.route("/gpsJSON")
def gpsJSON():
    DATA.gps.longitude += 0.00001
    if DATA.gps.longitude > -70.972903:
        DATA.gps.longitude = -70.986314
        DATA.gps.latitude += 0.0005
    randomOffset = 0.00005

    jsonDict = {
        "lat": DATA.gps.latitude + (random.random() * randomOffset) - randomOffset / 2,
        "lon": DATA.gps.longitude + (random.random() * randomOffset) - randomOffset / 2,
    }
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


@app.route("/map")
def root():
    return render_template("map.html")


def main():
    global DATA, app

    DATA = Website()
    port = int(os.environ.get("PORT", 5000))
    app.run(debug=True, host="0.0.0.0", port=port)


if __name__ == "__main__":
    main()
