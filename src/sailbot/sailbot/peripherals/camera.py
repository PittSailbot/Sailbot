"""
Interface for camera
"""

import math
import os
import time

import cv2
from rclpy.node import Node
from std_msgs.msg import String

from sailbot import constants as c
from sailbot.CV.objectDetection import ObjectDetection, draw_bbox
from sailbot.peripherals.cameraServos import CameraServos
from sailbot.utils.boatMath import distance_between
from sailbot.utils.utils import Waypoint


class Frame:
    """
    RGB image with sensor metadata frozen at the time of capture

    Attributes:
        - img (np.ndarray): the RGB image captured
        - time (float): time in seconds since 1970
        - gps (Waypoint): the camera's GPS position
        - heading: the camera's TRUE compass orientation
            - camera and boat can face in different directions (this var corrects for that)
        - pitch: the camera's pitch angle
        - detections: a list of buoy Detections
            - initially empty!
                - call objectDetection.analyze(Frame.img)
                - OR pass 'detect=True' on capture() or survey()
    """

    def __init__(self, img=None, time=None, gps=None, heading=None, pitch=None, detections=None):
        self.img = img
        self.time = time
        self.gps = gps
        self.heading = heading
        self.pitch = pitch
        self.detections = [] if detections is None else detections

    def __repr__(self):
        return f"Frame({self.img, self.time, self.gps, self.heading, self.pitch, self.detections})"


class Camera(Node):
    """
    Drivers and interface for camera

    Attributes:
        - servos (CameraServo): interface to control camera servos
            - servos.pitch and servos.yaw will always have the immediate camera position
            - pitch and yaw can be modified to move the physical servos

    Functions:
        - capture(): Takes a picture
        - survey(): Takes a panorama
    """

    def __init__(self):
        super().__init__("camera")
        self.logging = self._node.get_logger()

        self.path = os.getcwd()
        if c.config["MAIN"]["device"] == "pi":
            self.servos = CameraServos()
            self.gps_sub = self.create_subscription(String, "gps", self.gps_callback, 1)
            self.compass_sub = self.create_subscription(String, "compass", self.compass_callback, 1)

            self.position = None
            self.compass_angle = None
        else:
            self._cap = cv2.VideoCapture(int(c.config["CAMERA"]["source"]))

    def __del__(self):
        if c.config["MAIN"]["device"] != "pi":
            self._cap.release()

    def compass_callback(self, msg):
        self.compass_angle = int(msg.data())

    def gps_callback(self, msg):
        self.position = Waypoint.from_msg(msg)

    def capture(self, context=True, detect=False, annotate=False, save=False) -> Frame:
        """Takes a single picture from camera
        Args:
            - context (bool): whether to include time, gps, heading, and camera angle
            - detect (bool): whether to detect buoys within the image
            - annotate (bool): whether to draw detection boxes around the image
            - save (bool): whether to save the captured image
        Returns:
            - (camera.Frame): The captured image stored as a Frame object
        """

        frame = Frame()

        if c.config["MAIN"]["device"] == "pi":
            # Inefficient as FUCK
            cmd = rf"libcamera-still -t 1 -o '{self.path}/buffer0.jpg' --width 640 --height 640"
            os.system(cmd)
            frame.img = cv2.imread(f"{self.path}/buffer0.jpg")
            if frame.img is None:
                raise RuntimeError("No camera image detected!")
        else:
            _, frame.img = self._cap.read()

        if context:
            frame.time = time.time()
            frame.gps = Waypoint(self.position.longitude, self.position.latitude)
            frame.pitch = self.servos.pitch
            frame.heading = (self.compass_angle + (self.servos.yaw - 90)) % 360

        if detect:
            object_detection = ObjectDetection()
            frame.detections = object_detection.analyze(frame.img)
            if context:
                estimate_all_buoy_gps(frame)

            if annotate:
                draw_bbox(frame)

        if save:
            img_path = rf"{os.getcwd()}{c.config['MAIN']['log_path']}"
            filename = time.strftime("%m-%d %H-%M-%S", time.localtime(frame.time))
            full_path = rf"{img_path}\{filename}.png"

            i = 1
            while os.path.isfile(full_path):
                full_path = rf"{img_path}\{filename} {i}.png"
                i += 1

            cv2.imwrite(full_path, frame.img)

        return frame

    def survey(self, num_images=3, pitch=70, servo_range=180, context=True, detect=False, annotate=False, save=False) -> list[Frame]:
        """Takes a horizontal panaroma over the camera's field of view
            - Maximum boat FoV is ~242.2 degrees (not tested)
        # Args:
            - num_images (int): how many images to take across FoV
                - Picamera2 lens covers an FoV of 62.2 degrees horizontal and 48.8 vertical

            - pitch (int): fixed camera pitch angle
                - must be between 0 and 180 degrees: 0 points straight down, 180 points straight up

            - servo_range (int): the allowed range of motion for camera servos, always centered
                - must be between 0 and 180 degrees: 0 means servo is fixed to center, 180 is full servo range of motion
                - ex. a range of 90 degrees limits servo movement to between 45-135 degrees for a total boat FoV of 152.2 degrees

            - context (bool): whether to include time, gps and camera angle of captured images
            - detect (bool): whether to detect buoys in each image
            - annotate (bool): whether to draw bounding boxes around each detection
            - save (bool): whether to save the captured images
        # Returns:
            - list[camera.Frame]: A list of the captured images
        """

        images: list[Frame] = []
        servo_step = int(servo_range / num_images)
        MIN_ANGLE = int(c.config["CAMERASERVOS"]["min_angle"])
        MAX_ANGLE = int(c.config["CAMERASERVOS"]["max_angle"])

        # Move camera to desired pitch
        self.servos.pitch = pitch

        if self.servos.yaw <= 90:
            # Survey left -> right when camera is facing left or center
            for self.servos.yaw in range(MIN_ANGLE, MAX_ANGLE, servo_step):
                images.append(self.capture(context=context, annotate=annotate, save=save, detect=False))
        else:
            # Survey right -> left when camera is facing right
            for self.servos.yaw in range(MAX_ANGLE, MIN_ANGLE, servo_step):
                images.append(self.capture(context=context, annotate=annotate, save=save, detect=False))

        if detect:
            object_detection = ObjectDetection()
            for frame in images:
                frame.detections = object_detection.analyze(frame.img)

        return images

    def focus(self, detection) -> None:
        # TODO:
        #  - Bugfix code
        #  - Error check to raise Runtime exception when focusing is impossible
        #  - Overload to support focusing on GPS
        """Centers the camera on a detection to keep it in frame
        Args:
            - detection (Detection, Waypoint): the object to focus on

        Raises:
            - RuntimeError: when the camera cannot keep the object in frame using servos alone
                - NOTE: Should be pretty rare but occurs when trying to focus on something behind the boat
        """
        if type(detection) == Waypoint:
            self.logging.info(f"Focusing on GPS position: {detection}")
            # TODO
            distance = distance_between(self.gps, detection.GPS)
            boat_angle = self.compass_angle

            self.servos.yaw = 0
            self.servos.pitch = 70
        else:
            self.logging.info(f"Focusing on camera pixel detection")
            Cx, Cy = detection.x, detection.y
            Px, Py = Cx / c.config["OBJECTDETECTION"]["camera_width"], Cy / c.config["OBJECTDETECTION"]["camera_height"]
            if Px <= c.config["OBJECTDETECTION"]["center_acceptance"] and Py <= c.config["OBJECTDETECTION"]["center_acceptance"]:
                return

            """
            #find approriate amount turn based on pixels its behind by
            Tx,Ty = self.coordcalc(detection.w) #bad dist but useful
            if Cx < c.config["OBJECTDETECTION"]["camera_width"]/2: self.yaw(self.yaw)
            """
            # find approriate amount turn based by turning by regressive amounts if its too much
            turn_deg = 15
            if Cx - detection.w / 2 < 0:
                sign = -1  # left side
            else:
                sign = 1
            for i in range(5):  # after 5, fuck it
                self.servos.yaw = self.servos.yaw + sign * turn_deg

                # look (camera)
                frame = self.capture(detect=True, context=False)
                Cx, Cy = frame.detections[0].x, frame.detections[0].y
                Px, Py = (
                    Cx / c.config["OBJECTDETECTION"]["camera_width"],
                    Cy / c.config["OBJECTDETECTION"]["camera_height"],
                )
                if Px <= c.config["OBJECTDETECTION"]["center_acceptance"] and Py <= c.config["OBJECTDETECTION"]["center_acceptance"]:
                    break

                # TERRIBLE LOGIC
                if Cx - detection.w / 2 < 0:
                    signT = -1
                else:
                    signT = 1
                if sign * signT == -1:
                    turn_deg * 0.8


# TODO: Currently ignores camera height and pitch so estimated gps is based off of the triangle's leg vs hypotenuse
# Fix if estimated gps positions are innacurate
def estimate_all_buoy_gps(frame) -> None:
    """Approximates the locations of all detected buoys in a frame
        - Compares the ratio of buoy_size/distance to a fixed measured ratio
        - Uses camera angle and pixels from center to create a ray from the boat's current position
            - This ray is used to estimate the GPS point of each buoy
    # Args:
        - frame (camera.Frame):
    # Returns:
        - None
            - all frame.detections[].gps are updated
    """
    tested_width = float(c.config["OBJECTDETECTION"]["apparent_buoy_width_px"])
    tested_distance = float(c.config["OBJECTDETECTION"]["distance_from_buoy"])

    real_width = float(c.config["OBJECTDETECTION"]["real_buoy_width"])
    cam_center = int(c.config["CAMERA"]["resolution_width"]) / 2

    earth_radius = 6378000

    for detection in frame.detections:
        hypotenuse_distance = (tested_width / detection.w) * tested_distance
        dz = hypotenuse_distance * math.sin(frame.heading)  # IDK IF WORKS

        dx = (abs(detection.x - cam_center) / detection.w) * real_width
        dx *= math.cos(frame.heading)

        d_lat = (dz / earth_radius) * (180 / math.pi)
        d_lon = (dx / earth_radius) * (180 / math.pi) / math.cos(frame.gps.lat * math.pi / 180)

        lat = frame.gps.lat + d_lat
        lon = frame.gps.lon + d_lon

        detection.gps = Waypoint(lat, lon)
