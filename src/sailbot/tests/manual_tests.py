"""
Tests all necessary sensors and functionality of the boat
"""
import os
import warnings
from time import time

import cv2
import keyboard
import numpy as np

from sailbot.main import Boat
from sailbot.boatMovement import turn_to_angle
from sailbot.peripherals import camera
from sailbot.utils.utils import Waypoint

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if DOCKER == "True" else False

if not DOCKER:
    import rclpy
    from std_msgs.msg import String

    from sailbot.peripherals import compass, GPS, transceiver, windvane

    rclpy.init()


def manual_test_camera():
    """Manually test camera capture, servo movement and object detection
    Controls:
        - Arrow Keys (servo movement)
        - Enter (Take a picture and detect)
        - t (automatically track any detected buoys)
    """
    cam = camera.Camera()
    while True:
        print(f"Pitch: {cam.servos.pitch} Yaw: {cam.servos.yaw}\n")
        if keyboard.is_pressed("enter"):
            try:
                frame = cam.capture(context=True, detect=True, annotate=True)
            except Exception as e:
                warnings.warn(
                    f"""
                Failed to capture frame with context
                Exception Raised: {e}
                Attempting capture without context"""
                )
                frame = cam.capture(context=False, detect=True, annotate=True)

            print(f"Captured: {repr(frame)}")
            for detection in frame.detections:
                print(f"  {detection}")

        elif keyboard.is_pressed("space"):
            cam.servos.reset()

        elif keyboard.is_pressed("t"):
            while not keyboard.is_pressed("q"):
                frame = cam.capture(context=True, detect=True, annotate=True)

                print(f"Frame: {frame}")
                cv2.imshow("manual_test_camera()", frame.img)

                if len(frame.detections) != 0:
                    cam.focus(frame.detections[0])

        elif keyboard.is_pressed("up arrow"):
            cam.servos.pitch = cam.servos.pitch + 1
        elif keyboard.is_pressed("down arrow"):
            cam.servos.pitch = cam.servos.pitch - 1
        elif keyboard.is_pressed("left arrow"):
            cam.servos.yaw = cam.servos.yaw - 1
        elif keyboard.is_pressed("right arrow"):
            cam.servos.yaw = cam.servos.yaw + 1


def manual_test_cam_detect():
    """Infinitely runs camera detections and shows each detection
    - Good for measuring detection accuracy but not performance"""
    cam = camera.Camera()
    while True:
        start = time()
        frame = cam.capture(context=False, detect=True, annotate=True)

        end = time()
        fps = 1 / np.round(end - start, 2)
        cv2.putText(frame.img, f"FPS: {fps}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 2)
        cv2.imshow("manual_test_cam_detect()", frame.img)
        cv2.waitKey(1)


def manual_test_turn_to_angle():
    angle = 300
    print(f"pointing boat to {angle} degrees")
    turn_to_angle(angle)


def manual_test_go_to_gps():
    boat = Boat()

    destination = Waypoint(boat.gps.latitude, boat.gps.longitude)
    destination.add_meters(10, 10)
    print(f"Going to {destination}")
    boat.goToGPS(destination)


if __name__ == "__main__":
    choice = int(input("1: camera 2: go to gps 3: cam detect 4: turn to angle"))
    if choice == 1:
        manual_test_camera()
    elif choice == 2:
        manual_test_go_to_gps()
    elif choice == 3:
        manual_test_cam_detect()
    elif choice == 4:
        manual_test_turn_to_angle()
