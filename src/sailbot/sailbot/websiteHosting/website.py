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
import json
import math

import rclpy
from flask import Flask, render_template, request, jsonify, redirect
import socket
from geopy.distance import geodesic
from rclpy.node import Node
from std_msgs.msg import String, Float32
from rcl_interfaces.msg import Log
import sqlite3
from dateutil import parser

import sailbot.constants as c
from sailbot.utils.utils import DummyObject, Waypoint, ControlState, CameraServoState, EventLaunchDescription, create_directory_if_not_exists, ImuData
from sailbot.utils.boatMath import get_no_go_zone_bounds, is_within_angle, calculateCoordinates, remap

import os

MY_IP = '192.168.8.246'

app = Flask(__name__)
app.secret_key = "sailbot"

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if str(DOCKER).lower() == "true" else False
PI_DOCKER = os.environ.get("IS_PI_DOCKER", False)
PI_DOCKER = True if str(PI_DOCKER).lower() == "true" else False
if DOCKER:
    PORTS = os.environ.get("PORTS", "5000:5000")
    PORT = int(PORTS.split(':')[0])
    TILE_SERVER = "http://tile.openstreetmap.org/{z}/{x}/{y}.png"
elif PI_DOCKER:
    PORTS = os.environ.get("PORTS", "5000:5000")
    PORT = int(PORTS.split(':')[0])
    # IMPORTANT: Be sure to visit this address and accept the certificate if the map is not being displayed
    TILE_SERVER = 'https://' + MY_IP + ':443/tile/{z}/{x}/{y}.png'
    # an nginx container converts the images server by the OSM container on port 8080 to https server on port 443
else:
    raise Exception("configure ports and tile server")

log = logging.getLogger("werkzeug")
log.setLevel(logging.ERROR)


TIMEZONE = 'America/New_York'

RUDDER_MIN_ANGLE = int(c.config["RUDDER"]["min_angle"])
RUDDER_MAX_ANGLE = int(c.config["RUDDER"]["max_angle"])

SAIL_MIN_ANGLE = int(c.config["SAIL"]["min_angle"])
SAIL_MAX_ANGLE = int(c.config["SAIL"]["max_angle"])

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

        timer = self.parent.create_timer(0.5, retry_callback)
        
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

    def get_breadcrumbs_from_logs(self):
        query = 'SELECT * FROM logs WHERE level >= ?'
        params = [0]
        query += ' AND name LIKE ?'
        params.append(f'%GPS%')

        logs = self.get_logs(query, params)

        coords = []
        for log in logs:
            if log['msg'].startswith("GPS Publishing:"):
                gps_data = json.loads(log['msg'])
                coords.append(Waypoint(gps_data['lat'], gps_data['lon']))

        return coords

class Website(Node):
    def __init__(self):
        super().__init__("Website")
        self.logging = self.get_logger()
        self.logging.info(F"WEBSITE STARTED on port: {PORT}")
        threading.Thread(target=rclpy.spin, args=[self]).start()
        self.notification = ""
        
        self.createDummyObjs()

        self.dataDict = {
            "gps": f"{self.gps.latitude},{self.gps.longitude}",
            "compass": f"{self.compass.angle}",
            "odrive_axis0": f"{self.odrive.axis0.requested_state},{self.odrive.axis0.pos},{self.odrive.axis0.targetPos},{self.odrive.axis0.velocity},{self.odrive.axis0.currentDraw}",
            "odrive_axis1": f"{self.odrive.axis1.requested_state},{self.odrive.axis1.pos},{self.odrive.axis1.targetPos},{self.odrive.axis1.velocity},{self.odrive.axis1.currentDraw}",
        }

        self.logDB = LogDatabase(self)
        self.displayedBreadcrumbs = []#self.logDB.get_breadcrumbs_from_logs()
        

        self.waypoints = [
            {"name": "Waypoint 1", "lat": "42.849135", "lon": "-70.966314"},
            {"name": "Waypoint 2", "lat": "42.849135", "lon": "-70.996314"},
        ]

        self.circles = [{"lat": "42.849135", "lon": "-70.976314", "radius": 50}]

        self.boat_controlState = None
        self.boat_target = Waypoint(None, None)
        self.boat_event_coords = []
        self.warning_count = 0
        self.error_count = 0
        self.relative_wind = None
        self.sail_angle = 0.0
        self.rudder_angle = 0.0
        self.auto_sail_angle = 0.0
        self.auto_rudder_angle = 0.0
        self.imu = None

        self.camera_servo_pub = self.create_publisher(String, "/boat/cam_servo_control", 10)
        self.compass_offset_pub = self.create_publisher(Float32, "/boat/offset_compass", 10)
        self.setEventPub = self.create_publisher(String, "/boat/setEvent", 10)
        self.setEventTargetPub = self.create_publisher(String, "/boat/set_event_target", 10)

        # subscriptions should be started as the last step of init
        self.gps_subscription = self.create_subscription(
            String, "/boat/GPS", self.ROS_GPSCallback, 10
        )
        self.imu_subscription = self.create_subscription(
            String, "/boat/imu", self.ROS_imuCallback, 10
        )
        self.windvane_subscription = self.create_subscription(
            String, "/boat/wind_angle", self.ROS_windvaneCallback, 10
        )
        self.odrive_subscription = self.create_subscription(
            String, "/boat/odriveStatus", self.ROS_odriveCallback, 10
        )
        self.logMessages = self.create_subscription(
            Log, "/rosout", self.ROS_LogCallback, 10 # all log messages are published to this topic
        )
        self.boat_state_subscription = self.create_subscription(
            String, "/boat/next_gps", self.ROS_nextGpsCallback, 10
        )
        self.control_state_sub = self.create_subscription(
            String, "/boat/control_state", self.ROS_controlStateCallback, 2
        )
        self.queued_waypoints_subscription = self.create_subscription(
            String, "/boat/queued_waypoints", self.ROS_queuedWaypointsCallback, 10
        )
        self.sail_sub = self.create_subscription(Float32, "/boat/cmd_sail", self.ROS_sailCmd_callback, 10)
        self.rudder_sub = self.create_subscription(Float32, "/boat/cmd_rudder", self.ROS_rudderCmd_callback, 10)
        self.sail_sub = self.create_subscription(Float32, "/boat/cmd_auto_sail", self.ROS_sailAutoCmd_callback, 10)
        self.rudder_sub = self.create_subscription(Float32, "/boat/cmd_auto_rudder", self.ROS_rudderAutoCmd_callback, 10)

    def createDummyObjs(self):
        self.gps = DummyObject()
        self.gps.latitude = -1
        self.gps.longitude = -1
        self.gps.track_angle_deg = -1
        self.gps.velocity = 0

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

        if message.level >= 40:
            self.error_count += 1
        elif message.level >= 30:
            self.warning_count += 1

        self.logDB.insert_log(messageDict)

    def publishModeChange(self, modeString, file_path=None):
        
        try:
            msg = EventLaunchDescription(modeString, file_path).toRosMessage()
            self.setEventPub.publish(msg)
            self.logging.debug('Publishing: "%s"' % modeString) 
        except Exception as e:
            self.logging.error(e)

    def ROS_GPSCallback(self, string):
        string = string.data
        gpsJson = json.loads(string)

        lat, long = gpsJson['lat'], gpsJson['lon']
        self.gps.latitude = float(lat)
        self.gps.longitude = float(long)
        self.gps.track_angle_deg = float(gpsJson['track_angle'])
        self.gps.velocity = float(gpsJson['velocity'])

        self.dataDict["gps"] = f"{self.gps.latitude},{self.gps.longitude}"
        self.displayedBreadcrumbs.append(Waypoint(self.gps.latitude, self.gps.longitude))

    def ROS_imuCallback(self, string):
        data = ImuData.fromRosMessage(string)

        self.compass.angle = data.yaw
        self.imu = data

        self.dataDict["compass"] = f"{self.compass.angle}"

    def ROS_windvaneCallback(self, string):
        angle = float(string.data)
        self.relative_wind = angle

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

    def ROS_nextGpsCallback(self, msg):
        next_gps = Waypoint.from_msg(msg)
        # string = string.data
        # boatState = json.loads(string)

        self.boat_target = next_gps

    def ROS_controlStateCallback(self, msg):
        self.boat_controlState = ControlState.fromRosMessage(msg)
  
    def ROS_queuedWaypointsCallback(self, string):
        string = string.data
        coords = json.loads(string)

        self.boat_event_coords = []
        for waypoint in coords['Waypoints']:
            self.boat_event_coords.append(Waypoint.fromJson(waypoint))

    def ROS_sailCmd_callback(self, msg):
        self.sail_angle = float(msg.data)

    def ROS_rudderCmd_callback(self, msg):
        self.rudder_angle = float(msg.data)

    def ROS_sailAutoCmd_callback(self, msg):
        self.auto_sail_angle = float(msg.data)

    def ROS_rudderAutoCmd_callback(self, msg):
        self.auto_rudder_angle = float(msg.data)

@app.route("/", methods=["GET", "POST"])
def default():
    return redirect('/map')

@app.route("/home", methods=["GET", "POST"])
def home():
    return render_template("index.html", **DATA.dataDict)

@app.route("/camera", methods=["GET", "POST"])
def camera():
    if request.method == 'POST':
        pitch = request.form['pitch']
        yaw = request.form['yaw']
        msg = CameraServoState(yaw, pitch).toRosMessage()
        DATA.camera_servo_pub.publish(msg)

    return render_template("camera.html", video_url=F"https://{MY_IP}:8000/stream.mjpg")

@app.route("/offset_compass", methods=["GET", "POST"])
def compass_offset():
    if request.method == 'POST':
        offset = request.form['offset_compass']
        msg = Float32(data=offset)
        DATA.compass_offset_pub.publish(msg)

    return render_template("compass.html", video_url=F"https://{MY_IP}:8000/stream.mjpg")

@app.route("/mode/<mode>", methods=["GET", "POST"])
def setMode(mode):
    if 'file' not in request.files:
        file_path = None
    else:
        file = request.files['file']
        if file.filename == '':
            file_path = None
        else:
            create_directory_if_not_exists(f'uploaded_files/{file.filename}')
            file.save(f'uploaded_files/{file.filename}')
            file_path = f'uploaded_files/{file.filename}'

    mappingDict = {
        "manual": c.config["EVENTS"]["REMOTE_CONTROL"],
        "avoid": c.config["EVENTS"]["EVENT_COLLISION_AVOID"],
        "nav": c.config["EVENTS"]["EVENT_PRECISION_NAVIGATE"],
        "endurance": c.config["EVENTS"]["EVENT_ENDURANCE"],
        "keeping": c.config["EVENTS"]["EVENT_STATION_KEEPING"],
        "search": c.config["EVENTS"]["EVENT_SEARCH"],
    }

    if mode.lower() in mappingDict:
        DATA.publishModeChange(mappingDict[mode.lower()], file_path=file_path)
        DATA.notification = f"Mode set: {mode}, file:{file_path}"

    else:
        DATA.notification = f"ignoring attempt to set mode to unknown value"

    return {}

@app.route("/gps")
def gps():
    return f"{DATA.gps.latitude}, {DATA.gps.longitude}"

@app.route("/dataJSON")
def dataJSON():
    target = DATA.boat_target
    jsonDict = {
        "lat": DATA.gps.latitude, 
        "lon": DATA.gps.longitude, 
        "target_lat": target.lat, 
        "target_lon": target.lon,
        'warning_count': DATA.warning_count,
        "error_count": DATA.error_count,
        "ControlState": str(DATA.boat_controlState) if DATA.boat_controlState else "UNKNOWN",
        "queuedWaypoints": DATA.boat_event_coords,
        'relative_wind': DATA.relative_wind if DATA.relative_wind != None else 0.0,
        "compass_dir": DATA.compass.angle,
        'roll_dir': DATA.imu.roll if DATA.imu else 0.0,
        'pitch_dir': DATA.imu.pitch if DATA.imu else 0.0,
        "relative_target": calculate_cardinal_direction(DATA.gps.latitude, DATA.gps.longitude, target.lat, target.lon) - DATA.compass.angle if target.lat is not None else 0.0,
        "sail_angle": DATA.sail_angle,
        "rudder_angle": DATA.rudder_angle, #remap(DATA.rudder_angle, RUDDER_MIN_ANGLE, RUDDER_MAX_ANGLE, -90, 90),
        'speed': DATA.gps.velocity,
        'polygon_coords': get_no_go_zone_polygon(),
        'heading_polyline_coords': get_heading_coords(),
        'auto_sail' : DATA.auto_sail_angle,
        'auto_rudder' : DATA.auto_rudder_angle,
    }
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

@app.route("/breadcrumbs/<int:n>")
def breadcrumbs(n):
    if n > 0:
        recent_breadcrumbs = DATA.displayedBreadcrumbs[-n:]  # Get the last n breadcrumbs
    else:
        recent_breadcrumbs = DATA.displayedBreadcrumbs
    jsonDict = {"breadcrumbs": [wp.toJson() for wp in recent_breadcrumbs]}
    return jsonify(jsonDict)

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
    
@app.route('/setEventTarget', methods=['POST'])
def set_event_target():
    if request.method == 'POST':
        # Get form data from the request
        latitude = request.form.get('latitude')
        longitude = request.form.get('longitude')

        msg = Waypoint(latitude, longitude).to_msg()
        DATA.setEventTargetPub.publish(msg)

        return jsonify({'status': 'success', 'message': 'Set event published successfully'})

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

@app.route("/map")
def websiteMap():
    return render_template("map.html", tileServer=TILE_SERVER)

@app.route('/calculateDistance', methods=['GET'])
def calculate_distance():
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
            time.sleep(30)
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
    DATA.logging.info(F"Website available at https://localhost:{PORT}")

    # Generate the certificate using the following: openssl req -x509 -newkey rsa:4096 -nodes -out cert.pem -keyout key.pem -days 365
    app.run(debug=False, host="0.0.0.0", port=PORT, ssl_context=('cert.pem', 'key.pem')) # debug true causes the process to fork which causes problems

def calculate_cardinal_direction(lat1, lon1, lat2, lon2):
    # Convert latitude and longitude from degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    
    # Calculate the difference in longitude
    delta_lon = lon2_rad - lon1_rad
    
    # Calculate the y and x components of the directional vector
    y = math.sin(delta_lon) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
    
    # Calculate the angle in radians
    angle_rad = math.atan2(y, x)
    
    # Convert the angle from radians to degrees
    return math.degrees(angle_rad)

def get_no_go_zone_polygon():
    
    if DATA.relative_wind == None and DATA.compass.angle:
        return None
    try:
        left_bound, right_bound = get_no_go_zone_bounds(DATA.relative_wind, DATA.compass.angle)

        lx, ly = calculateCoordinates(DATA.gps.latitude, DATA.gps.longitude, left_bound, 300)
        rx, ry = calculateCoordinates(DATA.gps.latitude, DATA.gps.longitude, right_bound, 300)

        coords = [
            [DATA.gps.latitude, DATA.gps.longitude],
            [lx, ly],
            [rx, ry],
            [DATA.gps.latitude, DATA.gps.longitude]
        ]

        return coords
    except Exception as e:
        DATA.logging.warning(str(e))
        return None
    
def get_heading_coords():
    x, y = DATA.gps.latitude, DATA.gps.longitude
    hx, hy = calculateCoordinates(DATA.gps.latitude, DATA.gps.longitude, DATA.compass.angle, 150)

    return [[x,y], [hx, hy]]

if __name__ == "__main__":
    # main()
    pass
