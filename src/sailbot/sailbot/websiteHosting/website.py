# be sure to export FLASK_APP=websiteNoROS.py if you want this to run when doing 'flask run'

# run the exiting tileserver container with docker start <tile_server_name>
# make a new tile server using: sudo docker run -p 8080:80 -v osm-data:/data/database -d overv/openstreetmap-tile-server run
# see https://switch2osm.org/serving-tiles/using-a-docker-container/ 
# when starting the server it will take awhile before you can see anything

import os
import threading
import time
import re
from datetime import datetime, timedelta
import logging
import pytz

import rclpy
from flask import Flask, render_template, request, jsonify
import socket
from geopy.distance import geodesic
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import Log
import sqlite3
from dateutil import parser

import sailbot.constants as c
from sailbot.utils.utils import DummyObject


import os

app = Flask(__name__)
app.secret_key = "sailbot"

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if DOCKER == "True" else False
if DOCKER:
    PORTS = os.environ.get("PORTS", "5000:5000")
    PORT = int(PORTS.split(':')[0])
else:
    raise Exception("configure ports for pi")

log = logging.getLogger("werkzeug")
log.setLevel(logging.ERROR)

TILE_SERVER = 'http://' + 'localhost' + ':8080/tile/{z}/{x}/{y}.png'
TIMEZONE = 'America/New_York'

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

class LogDatabase:
    def __init__(self, parent, db_path='log_database.db'):
        self.db_path = db_path
        self.parent = parent
        self.create_table()

    def create_table(self):
        with self.get_connection() as connection:
            cursor = connection.cursor()
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS logs (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    level INTEGER,
                    msg TEXT,
                    name TEXT,
                    timestamp DATETIME,
                    line INTEGER,
                    function TEXT,
                    file TEXT
                )
            ''')
            connection.commit()
            cursor.close()

    def insert_log(self, message):
        
        message_tuple = (
            message['level'],
            message['msg'],
            message['name'],
            message['timestamp'],
            message['line'],
            message['function'],
            message['file']
        )

        max_retries = 10
        retry_count = 0
        def retry_callback():
            nonlocal retry_count
            nonlocal timer
            try:
                with self.get_connection() as connection:
                    cursor = connection.cursor()
                    cursor.execute('''
                        INSERT INTO logs (level, msg, name, timestamp, line, function, file)
                        VALUES (?, ?, ?, ?, ?, ?, ?)
                    ''', message_tuple)
                    connection.commit()
                    cursor.close()
                timer.cancel()
            except sqlite3.OperationalError as e:
                if "database is locked" in str(e):
                    retry_count += 1
                    if retry_count >= max_retries:
                        timer.cancel()
                        raise sqlite3.OperationalError("Failed to execute query after multiple retries.")
                else:
                    raise

        timer = self.parent.create_timer(0.1, retry_callback)
        
        

        
    def get_connection(self):
        return sqlite3.connect(self.db_path, check_same_thread=False)
    
    def get_all_logs(self, include_debug = False):
        with self.get_connection() as connection:
            cursor = connection.cursor()
            cursor.execute('SELECT * FROM logs WHERE level >= 20')
            rows = cursor.fetchall()
            cursor.close()

        logs = []
        for row in rows:
            log_dict = {
                'id': row[0],
                'level': row[1],
                'msg': row[2],
                'name': row[3],
                'timestamp': row[4],
                'line': row[5],
                'function': row[6],
                'file': row[7]
            }
            logs.append(log_dict)

        return logs

    def get_logs(self, *query):
        with self.get_connection() as connection:
            cursor = connection.cursor()
            cursor.execute(*query)
            rows = cursor.fetchall()
            cursor.close()

        logs = []
        for row in rows:
            log_dict = {
                'id': row[0],
                'level': row[1],
                'msg': row[2],
                'name': row[3],
                'timestamp': row[4],
                'line': row[5],
                'function': row[6],
                'file': row[7]
            }
            logs.append(log_dict)

        return logs


class Website(Node):
    def __init__(self):
        super().__init__("Website")
        self.logging = self.get_logger()
        self.logging.info(F"WEBSITE STARTED on port: {PORT}")
        threading.Thread(target=rclpy.spin, args=[self]).start()
        self.notification = ""
        
        self.createDummyObjs()

        self.modePub = self.create_publisher(String, "mode", 10)

        self.dataDict = {
            "gps": f"{self.gps.latitude},{self.gps.longitude}",
            "compass": f"{self.compass.angle}",
            "odrive_axis0": f"{self.odrive.axis0.requested_state},{self.odrive.axis0.pos},{self.odrive.axis0.targetPos},{self.odrive.axis0.velocity},{self.odrive.axis0.currentDraw}",
            "odrive_axis1": f"{self.odrive.axis1.requested_state},{self.odrive.axis1.pos},{self.odrive.axis1.targetPos},{self.odrive.axis1.velocity},{self.odrive.axis1.currentDraw}",
        }

        self.displayedBreadcrumbs = []
        self.logDB = LogDatabase(self)

        self.waypoints = [
            {"name": "Waypoint 1", "lat": "42.849135", "lon": "-70.966314"},
            {"name": "Waypoint 2", "lat": "42.849135", "lon": "-70.996314"},
        ]

        self.circles = [{"lat": "42.849135", "lon": "-70.976314", "radius": 50}]

        # subscriptions should be started as the last step of init
        self.gps_subscription = self.create_subscription(
            String, "/boat/GPS", self.ROS_GPSCallback, 10
        )
        self.compass_subscription = self.create_subscription(
            String, "/boat/compass", self.ROS_compassCallback, 10
        )
        self.odrive_subscription = self.create_subscription(
            String, "/boat/odriveStatus", self.ROS_odriveCallback, 10
        )

        self.logMessages = self.create_subscription(
            Log, "/rosout", self.ROS_LogCallback, 10 # all log messages are published to this topic
        )

    def createDummyObjs(self):
        self.gps = DummyObject()
        self.gps.latitude = -1
        self.gps.longitude = -1
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

    def addLogMessage(self, message):
        filename = message.file.replace('/workspace/install/sailbot/lib/sailbot/', '') if message.file.startswith('/workspace/install/sailbot/lib/sailbot/') else message.file

        messageDict = {
            'level': message.level,
            'msg': message.msg,
            'name': message.name,
            'timestamp': convert_to_datetime(message.stamp),
            'line': message.line,
            'function': message.function,
            'file': filename,
        }

        self.logDB.insert_log(messageDict)

    def ROS_GPSCallback(self, string):
        string = string.data
        print(F"gps string: {string}")
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
        self.displayedBreadcrumbs.append(Coordinate(self.gps.latitude, self.gps.longitude))

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

    def ROS_LogCallback(self, log):
        self.addLogMessage(log)

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

@app.route("/logs")
def logs(logMessages = None):
    level_conversions = {
        10: "10-DEBUG",
        20: "20-INFO",
        30: "30-WARNING",
        40: "40-ERROR",
        50: "50-FATAL",
    }
    if logMessages == None:
        logMessages = DATA.logDB.get_all_logs()

    for msg in logMessages:
        if 'level' in msg:
            msg['level'] = level_conversions[msg['level']] if msg['level'] in level_conversions else msg['level']

        msg['timestamp'] = convert_timestamp_to_local(msg['timestamp'])

    return render_template("logs.html", logMessages=logMessages)

@app.route('/log_search')
def log_search():
    return render_template('logSearch.html')

@app.route('/search_results', methods=['POST'])
def search_results():
    log_level = int(request.form['log_level'])
    start_time_str = request.form['start_time']
    end_time_str = request.form['end_time']
    file_filter = request.form['file']
    message_filter = request.form['message']
    function_filter = request.form['function']
    name_filter = request.form['name']

    # Convert start_time to a datetime object
    if start_time_str:
        user_input_start_time = parser.parse(start_time_str)
        timezone_ny = pytz.timezone('America/New_York')
        user_input_start_time_utc = timezone_ny.localize(user_input_start_time).astimezone(pytz.UTC)
    else:
        user_input_start_time_utc = None

    if end_time_str:
        user_input_end_time = parser.parse(end_time_str)
        timezone_ny = pytz.timezone('America/New_York')
        user_input_end_time_utc = timezone_ny.localize(user_input_end_time).astimezone(pytz.UTC)
    else:
        user_input_end_time_utc = None

    query = 'SELECT * FROM logs WHERE level >= ?'
    params = [log_level]

    if user_input_start_time_utc:
        query += ' AND timestamp >= ?'
        params.append(user_input_start_time_utc)

    if user_input_end_time_utc:
        query += ' AND timestamp <= ?'
        params.append(user_input_end_time_utc)

    if file_filter:
        query += ' AND file LIKE ?'
        params.append(f'%{file_filter}%')

    if message_filter:
        query += ' AND msg LIKE ?'
        params.append(f'%{message_filter}%')

    if name_filter:
        query += ' AND name LIKE ?'
        params.append(f'%{name_filter}%')

    if function_filter:
        query += ' AND function LIKE ?'
        params.append(f'%{function_filter}%')

    # display the logs
    return logs(DATA.logDB.get_logs(query, params))

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

@app.route("/map")
def websiteMap():
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

@app.route("/map")
def map():
    return render_template("map.html", tileServer=TILE_SERVER)

def convert_to_datetime(rosStamp):
    time_msg = rosStamp

    # Convert to Python datetime
    time_sec = time_msg.sec
    time_nsec = time_msg.nanosec

    # Combine seconds and nanoseconds into a single timestamp
    timestamp = time_sec + time_nsec / 1e9

    # Convert timestamp to datetime in UTC
    dt_utc = datetime.utcfromtimestamp(timestamp).replace(tzinfo=pytz.UTC)

    return dt_utc

def convert_timestamp_to_local(timestamp_utc_str):

    if timestamp_utc_str:
        # Convert timestamp string to datetime object
        timestamp_utc = parser.parse(timestamp_utc_str)

        # Convert UTC timestamp to local timezone
        local_timezone = pytz.timezone(TIMEZONE)  # Change to your local timezone
        timestamp_local = timestamp_utc.astimezone(local_timezone)

    return timestamp_local

def ros_main():
    global DATA, app

    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/website"
    rclpy.init()
    DATA = Website()

    app.run(debug=True, host="0.0.0.0", port=PORT)
    # rclpy.spin(DATA)


if __name__ == "__main__":
    # main()
    pass
