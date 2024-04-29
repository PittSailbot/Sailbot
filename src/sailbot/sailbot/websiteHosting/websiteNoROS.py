# be sure to export FLASK_APP=websiteNoROS.py if you want this to run when doing 'flask run'

# run the exiting tileserver container with docker start <tile_server_name>
# make a new tile server using: sudo docker run -p 8080:80 -v osm-data:/data/database -d overv/openstreetmap-tile-server run
# see https://switch2osm.org/serving-tiles/using-a-docker-container/ 
# when starting the server it will take awhile before you can see anything

import os
import random
import threading
import time

from flask import Flask, render_template, request, jsonify
from flask_cors import CORS

from geopy.distance import geodesic
import socket
from sailbot.sailbot.websiteHosting.networkLoggingServer import NetworkLoggingServer

def get_local_ip():
    # Create a socket to get the local IP address
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0.1)
    s.connect(('10.255.255.255', 1))  # Connect to a known address
    local_ip = s.getsockname()[0]
    s.close()
    return local_ip
    
TILE_SERVER = 'http://' + get_local_ip() + ':8080/tile/{z}/{x}/{y}.png'

app = Flask(__name__)
CORS(app) # not safe, should not be used on devices connected to the internet


class DummyObject:
    pass

class Coordinate:
    def __init__(self, lat, lon):
        self.lat = float(lat)
        self.lon = float(lon)

    def initFromGPS(gpsObj):
        return Coordinate(gpsObj.latitude, gpsObj.longitude)
    
    def __repr__(self) -> str:
        return F"Coordinate: ({self.lat},{self.lon})"
    
    def toJson(self):
        return {"lat": self.lat, "lon": self.lon}

class WebsiteData:
    def __init__(self):
        self.loggingServer = NetworkLoggingServer('0.0.0.0', 1234, callback=self.addLogMessage)
        self.gps = DummyObject()
        self.gps.latitude = 42.849135
        self.gps.longitude = -70.986314
        self.gps.track_angle_deg = -1

        self.compass = DummyObject()
        self.compass.angle = -1

        self.odrive = DummyObject()
        self.odrive.axis0 = DummyObject()
        self.odrive.axis0.requested_state = -1
        self.odrive.axis0.targetPos = -1
        self.odrive.axis0.pos = -1
        self.odrive.axis0.velocity = -1
        self.odrive.axis0.currentDraw = -1

        self.odrive.axis1 = DummyObject()
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

        self.displayedBreadcrumbs = []

        self.waypoints = [
            {"name": "Waypoint 1", "lat": "42.849135", "lon": "-70.966314"},
            {"name": "Waypoint 2", "lat": "42.849135", "lon": "-70.996314"},
        ]

        self.circles = [{"lat": "42.849135", "lon": "-70.976314", "radius": 50}]

        self.loggedMessages = []

        self.notification = ''

    def addLogMessage(self, message):
        level_conversions = {
            '20': "INFO"
        }
        parts = message.split(',')
        parts[0] = parts[0].split('(')[1]
        parts[-1] = parts[-1].replace(")", '')
        jsonizedMessage = {}
        for part in parts:
            key = part.split('=')[0].strip()
            value = part.split('=')[1].strip()
            if key == 'level' and str(value) in level_conversions:
                jsonizedMessage[key] = level_conversions[value]
            else:
                jsonizedMessage[key] = value

        self.loggedMessages.append(jsonizedMessage)
        print("added message", jsonizedMessage)

DATA = WebsiteData()

@app.route("/")
def home():
    return render_template("index.html", **DATA.dataDict)

@app.route("/gps")
def gps():
    return f"{DATA.gps.latitude}, {DATA.gps.longitude}"

@app.route("/dataJSON")
def dataJSON():

    

    DATA.gps.longitude += 0.00001
    if DATA.gps.longitude > -70.972903:
        DATA.gps.longitude = -70.986314
        DATA.gps.latitude += 0.0005
    randomOffset = 0.00005

    jsonDict = {
        "lat": DATA.gps.latitude + (random.random() * randomOffset) - randomOffset / 2,
        "lon": DATA.gps.longitude + (random.random() * randomOffset) - randomOffset / 2,
    }
    DATA.displayedBreadcrumbs.append(Coordinate(jsonDict['lat'], jsonDict['lon']))
    return jsonDict

@app.route("/logs")
def logs():
    print(DATA.loggedMessages)
    return render_template("logs.html", logMessages=DATA.loggedMessages)

@app.route("/test")
def test():
    return render_template("test.html", tileServer=TILE_SERVER)

@app.route("/breadcrumbs")
def breadcrumbs():
    jsonDict = {"breadcrumbs": [wp.toJson() for wp in DATA.displayedBreadcrumbs]}
    return jsonDict

@app.route("/waypoints")
def waypoints():
    jsonDict = {"waypoints": DATA.waypoints}
    return jsonDict

@app.route('/addWaypoint', methods=['POST'])
def add_waypoint():
    if request.method == 'POST':
        # Get form data from the request
        latitude = request.form.get('latitude')
        longitude = request.form.get('longitude')
        name = request.form.get('name')

        DATA.waypoints.append({"lat": latitude, "lon": longitude, "name": name})

        return jsonify({'status': 'success', 'message': 'Waypoint added successfully'})

@app.route('/addCircle', methods=['POST'])
def add_circle():
    if request.method == 'POST':
        # Get form data from the request
        latitude = request.form.get('latitude')
        longitude = request.form.get('longitude')
        radius = request.form.get('radius')

        DATA.circles.append({"lat": latitude, "lon": longitude, "radius": radius})

        return jsonify({'status': 'success', 'message': 'Circle added successfully'})

@app.route("/circles")
def circles():
    jsonDict = {"circles": DATA.circles}
    return jsonDict

@app.route("/compass")
def compass():
    return f"{DATA.compass.angle}"

@app.route("/map")
def map():
    return render_template("map.html", tileServer=TILE_SERVER)

@app.route('/calculateDistance', methods=['GET'])
def calculate_distance():
    print(request.args)
    try:
        # Get latitude and longitude of the selected waypoint
        selected_lat = float(request.args.get('selectedLat'))
        selected_lon = float(request.args.get('selectedLon'))

        # Get latitude and longitude of the target point
        target_lat = float(request.args.get('targetLat'))
        target_lon = float(request.args.get('targetLon'))

        # Calculate distance using the Haversine formula
        distance = geodesic((selected_lat, selected_lon), (target_lat, target_lon)).meters

        # You can replace the following line with your own logic to handle the calculated distance.
        return jsonify({'status': 'success', 'distance': distance})
    except:
        return jsonify({'status': 'failure'})

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


@app.teardown_appcontext
def teardown_appcontext(exception=None):
    DATA.loggingServer.stop_server()