import cv2
import numpy as np

# Trackbar callback function to update color range values
def callback(x):
    global color_space, low, high
    # Assign trackbar position value to low and high variables based on the selected color space
    if color_space == 'HSV':
        low = np.array([cv2.getTrackbarPos('low H', 'controls'), cv2.getTrackbarPos('low S', 'controls'), cv2.getTrackbarPos('low V', 'controls')], np.uint8)
        high = np.array([cv2.getTrackbarPos('high H', 'controls'), cv2.getTrackbarPos('high S', 'controls'), cv2.getTrackbarPos('high V', 'controls')], np.uint8)
    elif color_space == 'HLS':
        low = np.array([cv2.getTrackbarPos('low H', 'controls'), cv2.getTrackbarPos('low L', 'controls'), cv2.getTrackbarPos('low S', 'controls')], np.uint8)
        high = np.array([cv2.getTrackbarPos('high H', 'controls'), cv2.getTrackbarPos('high L', 'controls'), cv2.getTrackbarPos('high S', 'controls')], np.uint8)
    elif color_space == 'LAB':
        low = np.array([cv2.getTrackbarPos('low L', 'controls'), cv2.getTrackbarPos('low A', 'controls'), cv2.getTrackbarPos('low B', 'controls')], np.uint8)
        high = np.array([cv2.getTrackbarPos('high L', 'controls'), cv2.getTrackbarPos('high A', 'controls'), cv2.getTrackbarPos('high B', 'controls')], np.uint8)


# Create a separate window named 'controls' for trackbar
cv2.namedWindow('controls', cv2.WINDOW_NORMAL)
cv2.resizeWindow('controls', 550, 10)

# Global variables
low = np.array([0, 0, 0], np.uint8)
high = np.array([255, 255, 255], np.uint8)
color_space = 'HSV'

# Create trackbars for color range values
cv2.createTrackbar('low H', 'controls', 0, 179, callback)
cv2.createTrackbar('high H', 'controls', 179, 179, callback)
cv2.createTrackbar('low S', 'controls', 0, 255, callback)
cv2.createTrackbar('high S', 'controls', 255, 255, callback)
cv2.createTrackbar('low V', 'controls', 0, 255, callback)
cv2.createTrackbar('high V', 'controls', 255, 255, callback)
cv2.createTrackbar('low L', 'controls', 0, 255, callback)
cv2.createTrackbar('high L', 'controls', 255, 255, callback)
cv2.createTrackbar('low A', 'controls', 0, 255, callback)
cv2.createTrackbar('high A', 'controls', 255, 255, callback)
cv2.createTrackbar('low B', 'controls', 0, 255, callback)
cv2.createTrackbar('high B', 'controls', 255, 255, callback)

# Open webcam
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

while True:
    # Read webcam feed
    ret, frame = cap.read()

    if not ret:
        break

    # Convert frame to selected color space
    if color_space == 'HSV':
        color_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    elif color_space == 'HLS':
        color_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    elif color_space == 'LAB':
        color_img = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

    # Create a mask based on the color range
    mask = cv2.inRange(color_img, low, high)

    # Apply the mask to the original frame
    res = cv2.bitwise_and(color_img, color_img, mask=mask)

    # Show images
    cv2.imshow('mask', mask)
    cv2.imshow('res', res)

    # Wait for the user to press escape and break the while loop
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break
    # Change the color space based on the key pressed
    elif k == ord('1'):
        color_space = 'HSV'
    elif k == ord('2'):
        color_space = 'HLS'
    elif k == ord('3'):
        color_space = 'LAB'

# Release the webcam and destroy all windows
cap.release()
cv2.destroyAllWindows()
