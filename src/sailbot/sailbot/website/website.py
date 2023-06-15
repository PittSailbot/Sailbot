from flask import Flask, render_template
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time
import random

from sailbot.utils import dummyObject

app = Flask(__name__)


class Website(Node):
    def __init__(self):
        super().__init__('Website')
        self.logging = self.get_logger()
        self.logging.info("WEBSITE STARTED")
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

        self.gps_subscription = self.create_subscription(String, 'GPS', self.ROS_GPSCallback, 10)
        self.compass_subscription = self.create_subscription(String, 'compass', self.ROS_compassCallback, 10)
        self.odrive_subscription = self.create_subscription(String, 'odriveStatus', self.ROS_odriveCallback, 10)
        
        self.dataDict = {
            "gps": F"{self.gps.latitude},{self.gps.longitude}",
            "compass": F"{self.compass.angle}",
            "odrive_axis0": F"{self.odrive.axis0.requested_state},{self.odrive.axis0.pos},{self.odrive.axis0.targetPos},{self.odrive.axis0.velocity},{self.odrive.axis0.currentDraw}",
            "odrive_axis1": F"{self.odrive.axis1.requested_state},{self.odrive.axis1.pos},{self.odrive.axis1.targetPos},{self.odrive.axis1.velocity},{self.odrive.axis1.currentDraw}"
        }

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

        self.dataDict["gps"] = F"{self.gps.latitude},{self.gps.longitude}"

    def ROS_compassCallback(self, string):
        string = string.data
        if string == "None,None":
            self.compass.angle = None
            return

        angle = string.replace("(", "").replace(")", "")
        self.compass.angle = float(angle)

        self.dataDict["compass"] = F"{self.compass.angle}"

    def ROS_odriveCallback(self, string):
        string = string.data
        axis0Data = string.split(":")[0]
        axis1Data = string.split(":")[1]

        for axis, axisData in [(self.odrive.axis0, axis0Data), (self.odrive.axis1, axis1Data)]:
            axis.requested_state = axisData[0]
            axis.pos = axisData[1]
            axis.targetPos = axisData[2]
            axis.velocity = axisData[3]
            axis.currentDraw = axisData[4]

        self.dataDict["odrive_axis0"] = F"{self.odrive.axis0.requested_state},{self.odrive.axis0.pos},{self.odrive.axis0.targetPos},{self.odrive.axis0.velocity},{self.odrive.axis0.currentDraw}",
        self.dataDict["odrive_axis1"] = F"{self.odrive.axis1.requested_state},{self.odrive.axis1.pos},{self.odrive.axis1.targetPos},{self.odrive.axis1.velocity},{self.odrive.axis1.currentDraw}"

# @app.before_request
# def before_request():
#     rclpy.spin_once(DATA)  

@app.route('/')
def home():
    return render_template('index.html', **DATA.dataDict)

@app.route('/gps')
def gps():
    return (F"{DATA.gps.latitude}, {DATA.gps.longitude}")

@app.route('/compass')
def compass():
    return (F"{DATA.compass.angle}")

def main():
    global DATA, app
    
    os.environ['ROS_LOG_DIR'] = os.environ['ROS_LOG_DIR_BASE'] + "/website"
    rclpy.init()
    DATA = Website()
    port = int(os.environ.get('PORT', 5000))
    # threading.Thread(target=app.run, kwargs={'debug': True, 'host': '0.0.0.0', 'port': port}).start()
    threading.Thread(target=rclpy.spin, args=[DATA]).start()
    # threading.Thread(target=DATA.process).start()
    app.run(debug=True, host='0.0.0.0', port=port)
    # rclpy.spin(DATA)

if __name__ == "__main__":
    main()