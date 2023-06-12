import numpy as np
import cv2

def color_isolates(file):

    img = cv2.imread(file)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([0, 100, 120])
    upper = np.array([15, 255, 255])


    mask = cv2.inRange(hsv,lower,upper)
    return mask


#////////////////////////////
file = "pallet.jpg"

img = cv2.imread(file)
cv2.imshow("premask", img)

image = color_isolates(file)
cv2.imshow("org mask", image)

cv2.waitKey(0)
cv2.destroyAllWindows()