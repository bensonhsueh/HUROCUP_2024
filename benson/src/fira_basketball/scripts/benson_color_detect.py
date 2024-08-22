# print("a")
import numpy as np
# print("b")
import cv2
# print("c")
def empty(v):
    pass


# cap = cv2.VideoCapture("_video1.mp4")#get video
cap = cv2.VideoCapture(0)#get camera

cv2.namedWindow("track_bar")
cv2.resizeWindow("track_bar", 640,320)
cv2.createTrackbar("hue min", "track_bar",   0 , 179, empty)
cv2.createTrackbar("hue max", "track_bar", 179 , 179, empty)
cv2.createTrackbar("sat min", "track_bar",   0 , 255, empty)
cv2.createTrackbar("sat max", "track_bar", 255 , 255, empty)
cv2.createTrackbar("val min", "track_bar",   0 , 255, empty)
cv2.createTrackbar("val max", "track_bar", 255 , 255, empty)

while 1:
    # print(cap.read())
    if_get, img =  cap.read()#
    if if_get:
        # print("III")
        # img_blur = cv2.GaussianBlur(img, (3,3) , 5)
        # img = cv2.GaussianBlur(img, (5,5) , 5)#
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)#
        h_min = cv2.getTrackbarPos("hue min", "track_bar")
        h_max = cv2.getTrackbarPos("hue max", "track_bar")
        s_min = cv2.getTrackbarPos("sat min", "track_bar")
        s_max = cv2.getTrackbarPos("sat max", "track_bar")
        v_min = cv2.getTrackbarPos("val min", "track_bar")
        v_max = cv2.getTrackbarPos("val max", "track_bar")
        # h_min, h_max, s_min, s_max, v_min, v_max = 60,88,50,202,50,255
        print(h_min, h_max, s_min, s_max, v_min, v_max )
        lower = np.array([h_min,s_min,v_min])
        upper = np.array([h_max,s_max,v_max])

        mask_img = cv2.inRange(hsv_img, lower, upper)

        # cv2.resize("img", (640, 480))
        # cv2.resize(img, (0, 0), fx=0.5, fy=0.5)
        # img = cv2.resize(img, (0, 0), fx=0.2, fy=0.2)
        # mask_img = cv2.resize(mask_img, (0, 0), fx=0.2, fy=0.2)
        
        cv2.imshow("img",img)#show

        cv2.imshow("mask",mask_img)#show



    else:#finish or errror->no nest picture
        break
    cv2.waitKey(1)

