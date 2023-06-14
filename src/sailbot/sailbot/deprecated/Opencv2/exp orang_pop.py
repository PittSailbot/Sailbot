# import the libraries
import cv2 as cv
import numpy as np

# read the image
img = cv.imread("D://medium_blogs//red_coat.jpg")


vid_feed = cv.VideoCapture(0)

while True:
    _, img = vid_feed.read()

    # convert the BGR image to HSV colour space
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    # obtain the grayscale image of the original image
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # set the bounds for the red hue
    lower_red = np.array([5, 110, 150])
    upper_red = np.array([15, 255, 255])

    # create a mask using the bounds set
    mask = cv.inRange(hsv, lower_red, upper_red)
    # create an inverse of the mask
    mask_inv = cv.bitwise_not(mask)
    # Filter only the red colour from the original image using the mask(foreground)
    res = cv.bitwise_and(img, img, mask=mask)
    # Filter the regions containing colours other than red from the grayscale image(background)
    background = cv.bitwise_and(gray, gray, mask=mask_inv)
    # convert the one channelled grayscale background to a three channelled image
    background = np.stack((background,) * 3, axis=-1)
    # add the foreground and the background
    added_img = cv.add(res, background)

    # create resizable windows for the images
    cv.namedWindow("res", cv.WINDOW_NORMAL)
    cv.namedWindow("hsv", cv.WINDOW_NORMAL)
    cv.namedWindow("mask", cv.WINDOW_NORMAL)
    cv.namedWindow("added", cv.WINDOW_NORMAL)
    cv.namedWindow("back", cv.WINDOW_NORMAL)
    cv.namedWindow("mask_inv", cv.WINDOW_NORMAL)
    cv.namedWindow("gray", cv.WINDOW_NORMAL)

    # display the images
    cv.imshow("back", background)
    cv.imshow("mask_inv", mask_inv)
    cv.imshow("added", added_img)
    cv.imshow("mask", mask)
    cv.imshow("gray", gray)
    cv.imshow("hsv", hsv)
    cv.imshow("res", res)

    if cv.waitKey(0):
        cv.destroyAllWindows()

    # quits when pressing 'q'
    if cv.waitKey(1) & 0xFF == ord("q"):
        break
