# import the opencv library
import time

import cv2
import numpy as np


# ////////////////////////////////////////
def clr_iso(img, lower, upper):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    return mask


# ////////////////////////////////CONSTANTS////////////////////////////////

# ///////size contour cutoff///////
# 900 for jonah pc
SIZE = 900

# ////////color mask values////////
# BLUE / GREEN / RED

SELECT = 2
# 1: limited orange
# 2: big range orange
# 3: blue

if SELECT == 1:
    LOWER = np.array([5, 110, 150])
    UPPER = np.array([15, 255, 255])
elif SELECT == 2:
    LOWER = np.array([0, 110, 150])
    UPPER = np.array([30, 255, 255])
elif SELECT == 3:
    LOWER = np.array([101, 50, 38])
    UPPER = np.array([110, 255, 255])
else:
    exit("WRONG SELECT CONSTANT")

# //////////////////////////////////START//////////////////////////////////

# define a video capture object
vid_feed = cv2.VideoCapture(0)

count = 0
arr = []

# LOOP
while True:
    s_time = time.time()

    first = True
    real = False  # if there are contours

    # Capture the video frame by frame
    _, frame = vid_feed.read()
    #####////////////////////////////////////////
    #####obj detection

    # mask
    mask = clr_iso(frame, LOWER, UPPER)
    # make list of contours on screen
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # go through each contour
    for cnt in contours:
        # calc area and remove small elements
        area = cv2.contourArea(cnt)

        # CHANGE THIS VALUE IF FAR/CLOSE
        if area > SIZE:  # scale with camera
            if first:
                max_cnt = cnt
                first = False
                real = True
            elif area > cv2.contourArea(max_cnt):
                max_cnt = cnt
            # overall outline (VISUAL)
            cv2.drawContours(frame, [cnt], -1, (150, 0, 255), 2)

    # x, y, w, h = 0.0
    # perform VISUAL operations for max contour found in for statement
    if real:
        cv2.drawContours(frame, [max_cnt], -1, (255, 165, 0), 2)
        x, y, w, h = cv2.boundingRect(max_cnt)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)
        x1 = round(x + (w / 2))
        y1 = round(y + (h / 2))
        xystr = "xy:" + str(x) + ", " + str(y)
        whstr = "wh:" + str(w) + ", " + str(h)
        arstr = "max area: " + str(cv2.contourArea(max_cnt))
        cv2.putText(
            frame, xystr, (x - 50, y - 7), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2
        )  # xy
        cv2.putText(
            frame,
            whstr,
            (x - 50, y - 24),
            cv2.FONT_HERSHEY_PLAIN,
            1,
            (255, 255, 255),
            2,
        )  # wh
        cv2.putText(
            frame, "x", (x1, y1), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2
        )  # center
        cv2.putText(
            frame, arstr, (10, 20), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 2
        )  # area of max

    #####////////////////////////////////////////
    #####Display the resulting frame
    # (MAIN VISUAL)
    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)

    # TIME
    count += 1
    arr.append(time.time() - s_time)
    if count >= 100:
        print("mean:", np.mean(arr))
        count = 0
        arr = []
        # no detect: 0.024s
        # detect   : 0.025s to 0.030s

    # quits when pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# ////////////////////////////////////////
# clean up after loop
vid_feed.release()
cv2.destroyAllWindows()
