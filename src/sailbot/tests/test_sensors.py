"""
Tests all the boat's sensors
"""
import os
import warnings
from time import time

import cv2
import numpy as np
import pytest

from sailbot import constants as c
from sailbot.CV import objectDetection
from sailbot.peripherals import camera
from sailbot.utils.boatMath import distance_between
from sailbot.utils.utils import Waypoint

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if DOCKER == "True" else False

if not DOCKER:
    import rclpy
    from std_msgs.msg import String

    from sailbot.peripherals import compass, GPS, transceiver, windvane

    rclpy.init()


@pytest.mark.skipif(DOCKER, reason="only works on raspberry pi")
def test_gps():
    gps = GPS.GPS()

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

    tran = transceiver.Transceiver()

    for i in range(0, 3):
        tran.send("test")

        results = tran.read()
        print(f"Transceiver: {results}")
        assert results is not None


@pytest.mark.skipif(DOCKER, reason="only works on raspberry pi")
def test_camera_servos():
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

