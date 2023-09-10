# import the opencv library
import cv2
import numpy as np


# ////////////////////////////////////////
def clr_iso(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # BLUE / GREEN / RED
    lower = np.array([5, 110, 150])
    upper = np.array([15, 255, 255])

    # lower = np.array([101, 50, 38])
    # upper = np.array([110, 255, 255])

    mask = cv2.inRange(hsv, lower, upper)
    return mask


# ////////////////////////////////////////
# START


# define a video capture object
vid_feed = cv2.VideoCapture(0)


# ////////////////////////////////////////
# LOOP


while True:
    # Capture the video frame by frame

    _, frame = vid_feed.read()

    #####obj detection
    # mask = clr_iso(frame)
    # _, mask = cv2.threshold(mask, 254,255, cv2.THRESH_BINARY)

    mask = clr_iso(frame)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    arr = []
    for cnt in contours:
        # calc area and remove small elments
        area = cv2.contourArea(cnt)

        # CHANGE THIS VALUE IF FAR/CLOSE
        if area > 200:
            cv2.drawContours(frame, [cnt], -1, (255, 165, 0), 2)
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)

            xystr = str(x) + ", " + str(y)
            cv2.putText(
                frame, xystr, (x, y - 5), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2
            )

            arr.append([x + (w / 2), y + (h / 2)])

    avgx = 0
    avgy = 0
    count = len(arr)
    if count != 0:
        for x, y in arr:
            # print(x, y)
            avgx += x
            avgy += y
        avgx = round(avgx / count)
        avgy = round(avgy / count)
        print(avgx, avgy)
        cv2.putText(frame, "x", (avgx, avgy), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)

    # Display the resulting frame
    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)

    # quits when pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# ////////////////////////////////////////
# clean up after loop
vid.release()
cv2.destroyAllWindows()
