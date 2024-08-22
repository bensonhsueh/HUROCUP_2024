import numpy as np
import cv2 as cv

red_h_min = 0
red_h_max = 255
red_s_min = 0
red_s_max = 255
red_v_min = 0
red_v_max = 255

def nothing(x):
	pass

# rgb_image = cv.imread("Pictures/pic.jpeg")
# cap = cv.VideoCapture("/home/FIRA-Hurocup-Marathon-Vision/sources/videos/video2.mp4")
cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 160)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 120)
cv.namedWindow("Control")

cv.createTrackbar("H_min", "Control", 0, 179, nothing)
cv.createTrackbar("H_max", "Control", 0, 179, nothing)
cv.createTrackbar("S_min", "Control", 0, 255, nothing)
cv.createTrackbar("S_max", "Control", 0, 255, nothing)
cv.createTrackbar("V_min", "Control", 0, 255, nothing)
cv.createTrackbar("V_max", "Control", 0, 255, nothing)

cv.setTrackbarPos("H_min", "Control", 0)
cv.setTrackbarPos("H_max", "Control", 179)
cv.setTrackbarPos("S_min", "Control", 0)
cv.setTrackbarPos("S_max", "Control", 255)
cv.setTrackbarPos("V_min", "Control", 0)
cv.setTrackbarPos("V_max", "Control", 255)

while(True):
    ret, frame = cap.read() 
    # frame = rgb_image

    red_h_min = cv.getTrackbarPos("H_min", "Control")
    red_h_max = cv.getTrackbarPos("H_max", "Control")
    red_s_min = cv.getTrackbarPos("S_min", "Control")
    red_s_max = cv.getTrackbarPos("S_max", "Control")
    red_v_min = cv.getTrackbarPos("V_min", "Control")
    red_v_max = cv.getTrackbarPos("V_max", "Control")

    hsv_image = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    # th2 = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 10)
    lower_red = np.array([red_h_min, red_s_min, red_v_min])
    upper_red = np.array([red_h_max, red_s_max, red_v_max])
    red_bin_image = cv.inRange(hsv_image, lower_red, upper_red)

    # Display the resulting frame
    cv.imshow("frame", hsv_image)
    cv.imshow("mask", red_bin_image)
    if cv.waitKey(1) & 0xFF == ord("q"):
        break

# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
