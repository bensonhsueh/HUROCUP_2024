#! /usr/bin/env python
import rospy
from rospy.numpy_msg import numpy_msg
import numpy as np
from rospy_tutorials.msg import Floats
import cv2
import os

cap = cv2.VideoCapture(0)


seq = 401

while True:

    if os.path.isdir("dataset/seq"+str(seq)):
        seq+=1
    else:

        os.mkdir("dataset/seq"+str(seq))
        print("Now Collect ",seq)
        break




class Yolo_sub:


    def __init__(self):

        self.yolo_sub = rospy.Subscriber("yolo", numpy_msg(Floats),self.callback)
        # self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.Image_callback)
        self.label = []
        # self.get_pos =[]
        self.img_lst = []
        self.frame = 0
    def callback(self,data):

        try:



            self.frame+=1
            frame = self.frame
            img = self.img_lst[-1]
            self.label.append(data.data)
            # print()
            cv2.imwrite("dataset/seq"+str(seq)+"/"+str(frame)+"_"+str(data.data[0])+"_"+str(data.data[1])+"_"+str(data.data[2])+"_.jpg",img)
            
            # self.get_pos.append(True)

        except Exception as e:
            print(e)
            # pass
            # self.get_pos.append(False)

    # def Image_callback(self,img_data):

    #     bridge = CvBridge()

    #     try:
    #         img = bridge.imgmsg_to_cv2(img_data, "bgr8")

    #         self.img_lst.append(img)

    #     except:


    #         pass 



# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)



yolo_sub = Yolo_sub()

rospy.init_node('Yolo_sub', anonymous=True)

yolo_sub.yolo_sub
# yolo_sub.image_sub

# frame = 0

while not rospy.is_shutdown():

    _,img = cap.read()
    
    
    # img = cv2.resize(img,(640,480))

    # print(img.shape)

    try:
        # print(yolo_sub.label[-1])
        # cv2.putText(yolo_sub.img_lst[-1],"x: "+str(int(yolo_sub.label[-1][0]))+" y: "+str(int(yolo_sub.label[-1][1]))+" orientation: "+str(round(yolo_sub.label[-1][2],2)),(10,20),cv2.FONT_HERSHEY_PLAIN,2, (0, 0, 255), 1, cv2.LINE_AA)

        # cv2.imshow("camera",yolo_sub.img_lst[-1])
        yolo_sub.img_lst.append(img)
        # if yolo_sub.get_pos[-1] == True:
        #     cv2.putText(img,"x: "+str(int(yolo_sub.label[-1][0]))+" y: "+str(int(yolo_sub.label[-1][1]))+" orientation: "+str(round(yolo_sub.label[-1][2],2)),(10,20),cv2.FONT_HERSHEY_PLAIN,2, (0, 0, 255), 1, cv2.LINE_AA)
        
        # cv2.imwrite("dataset/seq26/"+str(frame)+"_"+str(yolo_sub.label[-1][0])+"_"+str(yolo_sub.label[-1][1])+"_"
        #             +str(yolo_sub.label[-1][2])+"_.jpg",img)

        cv2.imshow("camera",img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except Exception as e:
        print(e)