import numpy as np
import cv2 as cv

red_l_min = 0
red_l_max = 255
red_a_min = 0
red_a_max = 255
red_b_min = 0
red_b_max = 255

def nothing(x):
    pass

# rgb_image = cv.imread("Pictures/pic.jpeg")
cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640//2)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480//2)
cv.namedWindow("Control")

cv.createTrackbar("L_min", "Control", 0, 255, nothing)
cv.createTrackbar("L_max", "Control", 0, 255, nothing)
cv.createTrackbar("A_min", "Control", 0, 255, nothing)
cv.createTrackbar("A_max", "Control", 0, 255, nothing)
cv.createTrackbar("B_min", "Control", 0, 255, nothing)
cv.createTrackbar("B_max", "Control", 0, 255, nothing)

cv.setTrackbarPos("L_min", "Control", 0)
cv.setTrackbarPos("L_max", "Control", 255)
cv.setTrackbarPos("A_min", "Control", 0)
cv.setTrackbarPos("A_max", "Control", 255)
cv.setTrackbarPos("B_min", "Control", 0)
cv.setTrackbarPos("B_max", "Control", 255)

while True:
    ret, frame = cap.read()
    # /frame = rgb_image

    red_l_min = cv.getTrackbarPos("L_min", "Control")
    red_l_max = cv.getTrackbarPos("L_max", "Control")
    red_a_min = cv.getTrackbarPos("A_min", "Control")
    red_a_max = cv.getTrackbarPos("A_max", "Control")
    red_b_min = cv.getTrackbarPos("B_min", "Control")
    red_b_max = cv.getTrackbarPos("B_max", "Control")

    lab_image = cv.cvtColor(frame, cv.COLOR_BGR2LAB)

    lower_red = np.array([red_l_min, red_a_min, red_b_min])
    upper_red = np.array([red_l_max, red_a_max, red_b_max])
    red_bin_image = cv.inRange(lab_image, lower_red, upper_red)
    
    red_bin_image = cv.morphologyEx(red_bin_image, cv.MORPH_OPEN, np.ones((5, 5)))
    red_bin_image = cv.morphologyEx(red_bin_image, cv.MORPH_CLOSE, np.ones((5, 5)))

    # Display the resulting frame
    cv.imshow("frame", lab_image)
    cv.imshow("mask", red_bin_image)
    
    if cv.waitKey(1) & 0xFF == ord("q"):
        break

# cap.release()
cv.destroyAllWindows()
