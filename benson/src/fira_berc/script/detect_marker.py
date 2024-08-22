import rospy
import cv2
import numpy as np
from collections import deque
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2DArray, Detection2D
from sensor_msgs.msg import Image

class detect:
    id = 0
    score = 0
    center = [0, 0]
    size = [0, 0]

class Detectnet:
    def __init__(self, debug=False, verbose=1, n_class=4):
        self.verbose = verbose
        self.detect = [detect() for i in range(n_class)]
        for i in range(n_class):
            self.detect[i].id = i
        self.debug = debug
        if self.debug:
            self.bridge = CvBridge()
            rospy.Subscriber("/detectnet/overlay", Image, 
                self.overlay, queue_size=1)
    
    def read(self, ros_msg=None):
        temp = ros_msg.detections
        self.parse_data(temp)
    
    def overlay(self, ros_msg=None):
        try:
            img = self.bridge.imgmsg_to_cv2(ros_msg, "bgr8")
            cv2.imshow("marker", img)
            cv2.waitKey(1)
        except CvBridgeError:
            rospy.loginfo("err")
            print("error marker")
    
    def parse_data(self, detections):
        cnt = len(detections)
        temp_obj = [detect() for _ in range(cnt)]
        scores = {}
        for i in range(cnt):
            detection = detections[i].results[0]
            temp_obj[i].id = detection.id
            temp_obj[i].score = detection.score
            temp_obj[i].center = [detections[i].bbox.center.x, detections[i].bbox.center.y]
            temp_obj[i].size = [detections[i].bbox.size_x, detections[i].bbox.size_y]

            if temp_obj[i].id not in scores:
                scores[temp_obj[i].id] = temp_obj[i].score
            else:
                if temp_obj[i].score < scores[temp_obj[i].id]:
                    scores[temp_obj[i].id] = temp_obj[i].score

        max_id = max(temp_obj, key=lambda obj: obj.id).id
        if max_id >= len(self.detect):
            self.detect.extend([detect() for _ in range(max_id - len(self.detect) + 1)])

        for obj in temp_obj:
            if obj.score == scores[obj.id]:
                self.detect[obj.id] = obj
            else:
                obj.score = 0
            