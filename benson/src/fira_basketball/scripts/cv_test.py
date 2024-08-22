import cv2
import numpy as np

img = np.random.rand(480,640,3)


cv2.imshow("Random",img)
cv2.waitKey(0)