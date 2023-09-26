import time

import constants as c
import cv2
import numpy as np
from rclpy.node import Node


class camera:
    def __init__(self):
        self.vid_feed = cv2.VideoCapture(0)
        self.camera_width = float(self.vid_feed.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.camera_height = float(self.vid_feed.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self._node = Node("cameraDeprecated")
        self.logging = self._node.get_logger()

    def clr_iso(self, img, lower, upper):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        return mask

    def find_side(self, img, x):
        mid = img.get(cv2.CAP_PROP_FRAME_WIDTH) / 2

        if mid > x:
            return "LEFT"
        elif mid < x:
            return "RIGHT"
        else:
            return "CENTER"

    def run(self, type):
        while True:
            self.updateread(type)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        # clean up after loop
        self.vid_feed.release()
        cv2.destroyAllWindows()

    def updateread(self, type, preview=True):
        self._, self.frame = self.vid_feed.read()

        # comment out later with Machine Learning command
        if type == "filter":
            self.color_filter()
        else:
            self.logging.warning("Error camera mode, Attempt: " + type)

        if preview:
            # display in windows
            cv2.imshow("frame", self.frame)
            if self.low:
                cv2.imshow("mask", self.mask)

    # depreciated filter info for testing
    # clr_iso, find_side, color_filter
    def clr_iso(self, img, lower, upper):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        return mask

    def find_side(self, img, x):
        mid = img.get(cv2.CAP_PROP_FRAME_WIDTH) / 2

        if mid > x:
            return "LEFT"
        elif mid < x:
            return "RIGHT"
        else:
            return "CENTER"

    def color_filter(self):
        # ////////color mask values////////
        # BLUE / GREEN / RED

        SELECT = 3
        # 1: limited orange
        # 2: big range orange
        # 3: blue

        self.low = [0, 0, 0]
        self.up = [0, 0, 0]

        if SELECT == 1:
            self.low = [5, 110, 150]
            self.up = [50, 255, 255]
        elif SELECT == 2:
            self.low = [0, 110, 150]
            self.up = [30, 255, 255]
        elif SELECT == 3:
            self.low = [0, 90, 150]
            self.up = [110, 255, 255]
        elif SELECT == 4:
            self.low = [101, 50, 38]
            self.up = [110, 255, 255]
        elif SELECT == 5:
            self.low = [1, 80, 150]
            self.up = [100, 255, 255]
        else:
            exit("WRONG SELECT CONSTANT")

        LOWER = np.array(self.low)
        UPPER = np.array(self.up)

        cnt_count = 0
        s_time = time.time()

        first = True
        real = False  # if there are contours

        # mask
        self.mask = self.clr_iso(self.frame, LOWER, UPPER)
        # make list of contours on screen
        contours, self._ = cv2.findContours(
            self.mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        # cool boarder
        cv2.rectangle(self.frame, (5, 5), (210, 75), [50, 150, 255], -1)
        cv2.rectangle(self.frame, (5, 5), (210, 75), [0, 0, 0], 2)

        cv2.rectangle(self.frame, (5, 80), (30, 105), self.low, -1)
        cv2.rectangle(self.frame, (35, 80), (60, 105), self.up, -1)

        # go through each contour
        for cnt in contours:
            # calc area and remove small elements
            area = cv2.contourArea(cnt)

            # CHANGE THIS VALUE IF FAR/CLOSE
            if area > self.camera_width:  # scale with camera
                cnt_count += 1
                if first:
                    max_cnt = cnt
                    first = False
                    real = True
                elif area > cv2.contourArea(max_cnt):
                    max_cnt = cnt
                # overall outline (VISUAL)
                cv2.drawContours(self.frame, [cnt], -1, (150, 0, 255), 2)

        # x, y, w, h = 0.0
        # perform VISUAL operations for max contour found in for statement
        if real:
            cv2.drawContours(self.frame, [max_cnt], -1, (255, 165, 0), 2)
            x, y, w, h = cv2.boundingRect(max_cnt)
            cv2.rectangle(self.frame, (x, y), (x + w, y + h), (255, 0, 0), 3)
            x1 = round(x + (w / 2))
            y1 = round(y + (h / 2))
            xystr = "xy:" + str(x) + ", " + str(y)
            whstr = "wh:" + str(w) + ", " + str(h)
            arstr = "max area: " + str(cv2.contourArea(max_cnt))
            ctstr = "object cnt: " + str(cnt_count)

            if self.find_side(self.vid_feed, x + (w / 2)) == "RIGHT":
                cv2.putText(
                    self.frame,
                    xystr,
                    (x - 50, y - 7),
                    cv2.FONT_HERSHEY_PLAIN,
                    1,
                    (0, 0, 0),
                    2,
                )  # xy
                cv2.putText(
                    self.frame,
                    whstr,
                    (x - 50, y - 24),
                    cv2.FONT_HERSHEY_PLAIN,
                    1,
                    (255, 255, 255),
                    2,
                )  # wh
            else:
                cv2.putText(
                    self.frame,
                    xystr,
                    (x + 50, y - 7),
                    cv2.FONT_HERSHEY_PLAIN,
                    1,
                    (0, 0, 0),
                    2,
                )  # xy
                cv2.putText(
                    self.frame,
                    whstr,
                    (x + 50, y - 24),
                    cv2.FONT_HERSHEY_PLAIN,
                    1,
                    (255, 255, 255),
                    2,
                )  # wh

            cv2.putText(
                self.frame, "x", (x1, y1), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2
            )  # center
            cv2.putText(
                self.frame,
                ctstr,
                (10, 20),
                cv2.FONT_HERSHEY_PLAIN,
                1,
                (255, 255, 255),
                2,
            )  # num of cnts
            cv2.putText(
                self.frame,
                arstr,
                (10, 37),
                cv2.FONT_HERSHEY_PLAIN,
                1,
                (255, 255, 255),
                2,
            )  # area of max

            # perform REAL operations for max contour found in for statement
            cv2.putText(
                self.frame,
                "TARGET OUTPUT: " + str(x + (w / 2)),
                (10, 54),
                cv2.FONT_HERSHEY_PLAIN,
                1,
                (0, 0, 0),
                2,
            )  # TARGET cords
            cv2.putText(
                self.frame,
                self.find_side(self.vid_feed, x + (w / 2)),
                (10, 71),
                cv2.FONT_HERSHEY_PLAIN,
                1,
                (0, 0, 255),
                2,
            )  # TARGET output

        else:
            cv2.putText(
                self.frame,
                "NO CONTOURS",
                (10, 20),
                cv2.FONT_HERSHEY_PLAIN,
                1,
                (255, 255, 255),
                2,
            )  # area of max


if __name__ == "__main__":
    CAM = camera()
    while True:
        CAM.run("filter")
