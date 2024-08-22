#! /usr/bin/env python3
import os
import math
import rospy
import yaml
import numpy as np

from time import sleep
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32, Int32
from robot_msgs.msg import HeadMove, Button, ArucoData 
from robotis_controller_msgs.msg import StatusMsg
from op3_walking_module_msgs.msg import WalkingParam

from robotis_controller_msgs.srv import SetModule, SetJointModule, SetModule
from op3_action_module_msgs.srv import IsRunning


class Player:
    def __init__(self):
        rospy.init_node('sprint_demo')
        rospy.loginfo("[Player] Sprint Player - Running")
        
        self.freq       = 60.0
        self.Distance   = -1
        self.debug      = False
        
        ## state
        self.finish_ini = True ####
        self.walking    = False
        self.backward   = False
        
        ##  button 
        self.is_start = False
        
        ##  Subscriber
        rospy.Subscriber("/sprint/marker/position", ArucoData,	self.marker_pos_callback)
        rospy.Subscriber("/robotis/open_cr/button", String, self.button_callback)
        rospy.Subscriber("/robotis/status", StatusMsg, self.status_callback)
        rospy.Subscriber("/sprint/loss",String,self.loss_callback)
        ## Publisher
        # Pub status forward , stop , backward
        self.pub_scenario       = rospy.Publisher("/sprint/scenario", String,  queue_size=1)
        self.walk_command_pub   = rospy.Publisher('/robotis/walking/command', String, queue_size=1)
        
        ## Service
        self.set_present_ctrl_modules = rospy.ServiceProxy('/robotis/set_present_ctrl_modules', SetModule)
        
        while not self.finish_ini:
            rospy.Rate(self.freq).sleep()
        self.set_present_ctrl_modules("head_control_module")
        sleep(0.1)
        self.set_present_ctrl_modules("walking_module")
    
    def loss_callback(self,msg):
        data = msg.data
        if data == "back":
            self.backward = True
        
    def marker_pos_callback(self, msg):
        #  Aruco center marker
        # self.marker['x']    = msg.x
        # self.marker['y']    = msg.y
        
        # Distance
        self.Distance = msg.size
        
        if self.debug:
            rospy.loginfo('[Player] Distance: {}'.format(self.Distance))
        
    
    def button_callback(self,msg):
        rospy.loginfo('[Player] Button pressed: ' + msg.data)
        if msg.data == 'start' or msg.data == 'start_long':
            self.is_start = True
            rospy.loginfo("[Player] Start!! ")
            ### pub msgs to forward
            # self.walk_command_pub.publish('start')
        
        if msg.data == 'mode' or msg.data == 'mode_long':
            self.is_start = False
            rospy.loginfo("[Player] Stop!! ")
            ### pub msgs to stop
            self.pub_scenario.publish("STOP")
            self.walk_command_pub.publish('stop')
            
    def status_callback(self,msg):
        if msg.status_msg == 'Finish Init Pose':
            self.finish_ini = True
            rospy.loginfo("[Player] Finish Init Pose!! ")
    
    
    def run(self):
        input("Press to start")
        self.is_start = True  
        while not rospy.is_shutdown():
            
            if self.is_start:
                ## Forward
                if self.Distance > 80 and not self.backward:
                    self.pub_scenario.publish("forward")
                    if self.walking != True:
                        print('comecome')
                        self.walk_command_pub.publish('start')
                        self.walking = True
                
                ## Stop_Forward and Start_Backward
                elif self.Distance < 80 and not self.backward and self.Distance != -1:
                    self.pub_scenario.publish("STOP")
                    self.walk_command_pub.publish('stop')
                    sleep(2)
                    self.pub_scenario.publish("backward")
                    self.backward = True
                    self.walk_command_pub.publish("start")
                    rospy.loginfo("[Player] Start Backward")
                elif self.backward and self.Distance > 250 :
                    self.pub_scenario.publish("init")
                    self.walk_command_pub.publish('stop')
                    input("Press to start")
                    self.backward = False                    
                    self.walking = False
                    
            # if kbhit():
            #     self.is_start = False
            #     self.walk_command_pub.publish('stop')
                
            rospy.Rate(self.freq).sleep()
        
if __name__ == "__main__":
    player = Player()
    player.run()