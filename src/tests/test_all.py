"""
Tests all necessary sensors and functionality of the boat
"""
import os
import warnings
from time import time

import cv2
import keyboard
import numpy as np
import pytest

import sailbot.boatMain as boatMain
from sailbot import constants as c
from src.sailbot.CV import objectDetection
from sailbot.peripherals import camera
from src.sailbot.utils import distance_between
from src.sailbot.utils import Waypoint

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if DOCKER == "True" else False

if not DOCKER:
    import rclpy

    import sailbot.peripherals.compass as compass
    import sailbot.peripherals.GPS as GPS
    import src.sailbot.peripherals.transceiver as transceiver
    import src.sailbot.peripherals.windvane as windvane

    rclpy.init()

# ----------------------------------  BASIC  ----------------------------------
def test_mainInit():
    boat = boatMain.boat(calibrateOdrive=False)


def test_eventInit():
    boat = boatMain.boat(calibrateOdrive=False)

    _node = rclpy.Node()
    _node.create_publisher(String, "mode", 10)
    modes = [
        c.config["MODES"]["MOD_RC"],
        c.config["MODES"]["MOD_COLLISION_AVOID"],
        c.config["MODES"]["MOD_PRECISION_NAVIGATE"],
        c.config["MODES"]["MOD_ENDURANCE"],
        c.config["MODES"]["MOD_STATION_KEEPING"],
        c.config["MODES"]["MOD_SEARCH"],
    ]
    for mode in modes:
        msg = String()
        msg.data = f"Set Mode:{mode}"
        _node.publish(msg)
        rclpy.spin_once(boat)
        assert boat.modeSetting == mode


# ---------------------------------- SENSORS ----------------------------------

@pytest.mark.skipif(DOCKER, reason="only works on raspberry pi")
def test_gps():
    gps = GPS.gps()

    for i in range(0, 3):
        results = (gps.latitude, gps.longitude)
        print(f"GPS: ({results[0]}, {results[1]})")
        assert results[0] is not None


@pytest.mark.skipif(DOCKER, reason="only works on raspberry pi")
def test_compass():
    comp = compass.Compass()

    for i in range(0, 3):
        results = comp.angle
        print(f"Compass: {results})")
        assert results is not None

    comp.destroy_node()
    rclpy.shutdown()


@pytest.mark.skipif(DOCKER, reason="only works on raspberry pi")
def test_windvane():
    wv = windvane.WindVane()

    for i in range(0, 3):
        results = wv.position
        print(f"Windvane: {results}")
        assert results is not None


@pytest.mark.skipif(DOCKER, reason="only works on raspberry pi")
def test_transceiver():
    ardu = transceiver.arduino(c.config["MAIN"]["ardu_port"])

    for i in range(0, 3):
        results = ardu.readData()
        print(f"Transceiver: {results}")
        assert results is not None


@pytest.mark.skipif(DOCKER, reason="only works on raspberry pi")
def test_servos():
    servos = camera.CameraServos()

    servos.pitch = 70
    assert 69 < servos.pitch < 71, f"Camera servo pitch outside of acceptable error, expected 70, got: {servos.pitch}"

    servos.yaw = 2000
    assert 179 < servos.yaw < 181, f"Camera servo yaw unprotected from impossible range, expected 180, got: {servos.yaw}"


def test_cam_detect():
    """Validity and performance test for buoy detection"""
    cam = camera.Camera()
    NUM_CAPTURES = 5
    test_start = time()

    for i in range(0, NUM_CAPTURES):
        frame = cam.capture(context=False, detect=True, save=True)
        assert frame is not None and frame.img is not None, f"Camera capture returned {frame}, expected image"

    test_end = time()
    avg_fps = NUM_CAPTURES / np.round(test_end - test_start, 2)

    print(f"\nAverage FPS: {avg_fps}")
    if avg_fps < 0.5:
        with pytest.warns(UserWarning):
            warnings.warn(f"Low average FPS ({avg_fps}) for detections")


def test_img_detect(img=rf"{c.root_dir}\data\CV\test_buoy.jpg"):
    """Detects buoys from specified image path(s)
    Args:
        img (str): file path of selected image
    """
    # Basic model function can be tested by running `yolo predict model=CV/buoy_weights.pt source=0`
    object_detection = objectDetection.ObjectDetection()

    object_detection.model.predict(
        source=img,
        show=True,
        conf=float(c.config["OBJECTDETECTION"]["conf_thresh"]),
        save=False,
        line_thickness=1,
    )


@pytest.mark.skipif(DOCKER, reason="only works on raspberry pi")
def test_gps_estimation():
    """Prints out simulated gps locations for test_buoy.jpg"""
    object_detection = objectDetection.ObjectDetection()

    sim_frame = camera.Frame(
        gps=Waypoint(0, 0),
        heading=90,
        pitch=45,
        detections=object_detection.analyze("sailbot/CV/test_buoy.jpg"),
    )

    camera.estimate_all_buoy_gps(sim_frame)
    for detection in sim_frame.detections:
        print(f"Detection at {detection.GPS}, {distance_between(Waypoint(0, 0), detection.GPS)}")


@pytest.mark.skipif(DOCKER, reason="only works on raspberry pi")
def test_survey():
    cam = camera.Camera()

    images = cam.survey(context=True, detect=True, annotate=True, pitch=70, save=True)
    assert images is not None
    for frame in images:
        cv2.imshow("test_survey()", frame.img)


# ---------------------------------- CONTROLS ----------------------------------


@pytest.mark.skipif(DOCKER, reason="only works on raspberry pi")
def test_rudder():
    boat = boatMain.boat()
    boat.turnToAngle(30, wait_until_finished=True)
    assert boat.currentRudder == 30


@pytest.mark.skip(reason="Not implemented")
@pytest.mark.skipif(DOCKER, reason="only works on raspberry pi")
def test_sail():
    pass


# -------------------------------- MANUAL TESTS --------------------------------


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


def manual_test_go_to_gps():
    boat = boatMain.boat()

    destination = Waypoint(boat.gps.latitude, boat.gps.longitude)
    destination.add_meters(10, 10)
    print(f"Going to {destination}")
    boat.goToGPS(destination)


if __name__ == "__main__":
    choice = int(input("1: camera 2: go to gps 3: cam detect"))
    if choice == 1:
        manual_test_camera()
    elif choice == 2:
        manual_test_go_to_gps()
    elif choice == 3:
        manual_test_cam_detect()
