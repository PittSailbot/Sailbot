# import the opencv library
import cv2
import numpy as np
import time


# ////////////////////////////////////////
def clr_iso(img, lower, upper):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    return mask


def find_side(img, x):
    mid = img.get(cv2.CAP_PROP_FRAME_WIDTH) / 2

    if mid > x:
        return "LEFT"
    elif mid < x:
        return "RIGHT"
    else:
        return "CENTER"


# ////////////////////////////////CONSTANTS////////////////////////////////

# ///////size contour cutoff///////
# 900 for jonah pc
SIZE = 900
# modeled with bottom of soldering plate with the 4 arm thingies
# hold with dots horizontal (upright paperwise)
# length in mm
Wid_real = 14  # find model
Focal_L = (30 * 310) / Wid_real  # (W*P_ex)/D_ex#calc (find pixels found and real distance in a case then plug in)

# ////////color mask values////////
# BLUE / GREEN / RED

SELECT = 2
# 1: limited orange
# 2: big range orange (use for distance)
# 3: blue

if SELECT == 1:
    low = [5, 110, 150]
    up = [50, 255, 255]
elif SELECT == 2:
    low = [0, 110, 150]
    up = [30, 255, 255]
elif SELECT == 3:
    low = [0, 90, 150]
    up = [110, 255, 255]
elif SELECT == 4:
    low = [101, 50, 38]
    up = [110, 255, 255]
elif SELECT == 5:
    low = [1, 80, 150]
    up = [100, 255, 255]
else:
    exit("WRONG SELECT CONSTANT")

LOWER = np.array(low)
UPPER = np.array(up)

# //////////////////////////////////START//////////////////////////////////

# define a video capture object
vid_feed = cv2.VideoCapture(0)

count = 0
arr = []

# LOOP
while True:
    cnt_count = 0
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

    # cool boarder
    cv2.rectangle(frame, (5, 5), (210, 92), [50, 150, 255], -1)
    cv2.rectangle(frame, (5, 5), (210, 92), [0, 0, 0], 2)

    # sample rec
    cv2.rectangle(frame, (5, 100), (30, 122), low, -1)
    cv2.rectangle(frame, (35, 100), (60, 122), up, -1)

    # TO DO WORK:
    """
    - detect contours inside box of other contours and grouping them together
    - have a one use setup func at start of prog and when there hasnt been a new spotting in a while
      to change the color mask based of object recognition and taking its color values
    """

    # go through each contour
    for cnt in contours:
        # calc area and remove small elements
        area = cv2.contourArea(cnt)

        # CHANGE THIS VALUE IF FAR/CLOSE
        if area > SIZE:  # scale with camera
            cnt_count += 1
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
        ctstr = "object cnt: " + str(cnt_count)

        if find_side(vid_feed, x + (w / 2)) == "RIGHT":
            cv2.putText(frame, xystr, (x - 50, y - 7), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2)  # xy
            cv2.putText(frame, whstr, (x - 50, y - 24), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 2)  # wh
        else:
            cv2.putText(frame, xystr, (x + 50, y - 7), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2)  # xy
            cv2.putText(frame, whstr, (x + 50, y - 24), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 2)  # wh

        cv2.putText(frame, "x", (x1, y1), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)  # center
        cv2.putText(frame, ctstr, (10, 20), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 2)  # num of cnts
        cv2.putText(frame, arstr, (10, 37), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 2)  # area of max

        # perform REAL operations for max contour found in for statement
        cv2.putText(
            frame, "TARGET OUTPUT(midx): " + str(x + (w / 2)), (10, 54), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2
        )  # TARGET cords
        cv2.putText(
            frame, find_side(vid_feed, x + (w / 2)), (10, 71), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2
        )  # TARGET output
        # print( find_side(vid_feed, x+(w/2)) )

        # distance formula
        D = (Wid_real * Focal_L) / w
        cv2.putText(frame, "DISTANCE: " + str(D), (10, 88), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)  # TARGET output

    else:
        cv2.putText(frame, "NO CONTOURS", (10, 20), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 2)  # area of max

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
