"""
Interface for detecting buoys
"""
from ultralytics import YOLO  # Documentation: https://docs.ultralytics.com/cfg/
# from supervision.tools.detections
import cv2
import numpy as np
import torch
import logging
from dataclasses import dataclass

from sailbot import constants as c


@dataclass(order=True)
class Detection:
    """
    Object containing the confidence level, bounding box, and location of a buoy from a given image
    Attributes:
        - x (int): - x coordinate for the center of the buoy
        - y (int): - y coordinate for the center of the buoy
        - w (int): - width (in pixels) of bounding box rectangle
        - h (int): - height (in pixels) of bounding box rectangle
        - conf (float): - confidence level that detected object is a buoy [0-1]
        - self.gps (Waypoint) - approximate gps position of buoy
    """
    
    def __init__(self, result: torch.tensor):
        _bbox: torch.tensor = result.boxes
        _xywh: list[float] = np.rint(_bbox.xywh[0]).tolist()

        self.x = int(_xywh[0])
        self.y = int(_xywh[1])
        self.w = int(_xywh[2])
        self.h = int(_xywh[3])
        self.conf = round(_bbox.conf.item(), 2)
        # self.class_id: str = ObjectDetection.classes[int(_bbox.cls.numpy()[0])]
        self.gps = None
        sort_index: int = self.conf

    def __str__(self):
        return f"Detection ({self.conf}%) at ({self.x}, {self.y}) with width {self.w}px and height {self.h}px"
        

class ObjectDetection:
    """
    AI object detection model
    
    Functions: 
        - analyze() - checks image for buoys
    """
    def __init__(self):
        self.model = YOLO(c.root_dir + c.config["OBJECTDETECTION"]["weights"])  # Initialize model for analysis
            # TODO: test performance after export to .onnx
            #model.export(format="onnx") or cmd -> yolo task=detect mode=export model=<PATH> format = onnx
    
    def analyze(self, image) -> list[Detection]:
        """Detects buoys within a given image. Can be supplied from cv2 or using the Camera.capture()/Camera.survey() methods.
        Args:
            - image (np.ndarray, .jpg, .png): The RGB image to search for buoys
        Returns:
            - A list buoys found (if any) stored as a list of Detection objects
                - list is sorted by highest confidence, detections[0] is ALWAYS the highest confidence match
        """
        # TODO: test results.cpu() or results.to("cpu") for performance on Pi
        result = self.model.predict(source=image, conf=float(c.config["OBJECTDETECTION"]["conf_thresh"]), save=False, line_thickness=1)
        result = result[0] # metadata -> list[tensor]
        
        # Add each buoy found by the model into a list
        detections: list[Detection] = []
        for detection in result:
            logging.info("Buoy ({detection.conf}): at ({detection.x},{detection.y})\n")
            detections.append(Detection(detection)) # Convert tensors into readable Detection class and append to list
        detections.sort(reverse=True)
        return detections


def draw_bbox(frame):
    """Draws bounding boxes around each detection in a frame
    Args:
        - frame (camera.Frame)
    """
    for detection in frame.detections:
        x, y, w, h = detection.x, detection.y, detection.w, detection.h
        cv2.rectangle(img=frame.img,
                      pt1=(int(x - w / 2), int(y + h / 2)),
                      pt2=(int(x + w / 2), int(y - h / 2)),
                      color=(0, 255, 0),
                      thickness=2)
        cv2.putText(img=frame.img,
                    text=f'Buoy ({detection.conf})',
                    org=(int(x - w / 2), int(y + h / 2) + 15),
                    fontFace=0,
                    fontScale=0.4,
                    color=(0, 255, 0))
