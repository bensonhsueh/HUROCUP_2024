#! /usr/bin/env python
import rospy
from op3_ros_utils import *
import cv2
from rospy.numpy_msg import numpy_msg
import numpy as np
from rospy_tutorials.msg import Floats

rospy.init_node("Sprint_VL")



robot = Robot()

def init():
    # Set ctrl modules of all actions to joint, so we can reset robot position

    robot.setJointsControlModule(["head_tilt"], ["none"])
    robot.setGeneralControlModule("walking_module")

init()


     


class VL_sub:


    def __init__(self):

        self.pos_sub = rospy.Subscriber("/Localization", numpy_msg(Floats),self.callback)
        self.pos_pred_lst = []
    def callback(self,data):

        try:




            self.pos_pred_lst.append(data.data)
            

        except Exception as e:
            print(e)

vl_sub = VL_sub()

vl_sub.pos_sub



go = raw_input("Go")

rospy.sleep(3)   

if go == "y":


    robot.walkStart()

    while not rospy.is_shutdown():


        try:


            pos_pred = vl_sub.pos_pred_lst[-1]

            
            origentation = pos_pred[2]

            print(pos_pred)

            speed = 10

            if pos_pred[0]>=400:

                speed = 5

            else:

                speed = 10

            x = (pos_pred[0]-500)



            if x > 0:

                robot.walkVelocities(x=int(speed),th=float(1.5), balance=True, hip_pitch=8.0) #36

                rospy.sleep(4)

                robot.walkStop()

                rospy.sleep(5)

                # print(pos_pred)
                break

            else:

                speed *= 1


            if abs(90 - origentation)>=10:



                if (90 - origentation)>0:

                    theta = 3.0

                  
                    print("turn left")

                else:

                    theta = 1.0

                    print("trun right")

    

            else:
                print("Go Stright")
                theta = 2.0



            robot.walkVelocities(x=int(speed),th=float(theta), balance=True, hip_pitch=8.0) #36

     
            # rospy.sleep(walk_time)
            # robot.walkStop()
            # rospy.sleep(3)

            if robot.buttonCheck("mode"):

                robot.walkStop()

                break

        except Exception as e:

            print(e)




    # rospy.sleep(5)

    print("Prepare Backward")

    while not rospy.is_shutdown():


        try:


            pos_pred = vl_sub.pos_pred_lst[-1]

            
            origentation = pos_pred[2]

            print(pos_pred)

            speed = -15

            if pos_pred[0]<=300:

                speed = -11

            else:

                speed = -15

            x = (pos_pred[0]-50)



            if x < 0:

                robot.walkVelocities(x=int(speed),th=float(2.5),balance=True, hip_pitch=4) #36

                rospy.sleep(5)

                robot.walkStop()

                print(pos_pred)
                break

            else:

                speed *= 1


            if (90 - origentation)>0:

                theta = 6.0


                print("turn left")

            else:

                theta = -0.5

                print("trun right")

  

   

            robot.walkVelocities(x=int(speed),th=float(theta), balance=True, hip_pitch=8.0) #36

            robot.walkStart()


     
            # rospy.sleep(walk_time)
            # robot.walkStop()
            # rospy.sleep(3)

            if robot.buttonCheck("mode"):

                robot.walkStop()

                break

        except Exception as e:

            print(e)



























else:



    print("STOP")
