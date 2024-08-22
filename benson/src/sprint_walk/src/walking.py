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

from sensor_msgs.msg import JointState
from robotis_controller_msgs.srv import SetModule, SetJointModule, SetModule
from op3_action_module_msgs.srv import IsRunning

class Walking():
    def __init__(self):
        rospy.init_node('sprint_walk')
        rospy.loginfo("[Walking] Walking Control - Running")
        print('lkk')
        self.debug = False
        
        self.freq       = 60.0
        self.main_rate  = rospy.Rate(self.freq)
        self.frame_w    = 640
        self.frame_h    = 480
        self.marker     = { 'x': -1, 'y': -1, 'size': -1 }
        self.tracking   = False
        
        ##  head
        self.head   = { 'pan': 0.0,  'tilt': 0.0}
        self.pan    = { 'min': -2.0, 'max':  2.0}
        self.tilt   = { 'min': -1, 'max': -np.radians(0)}
        self.pos_tilt   = np.radians(-5)

        self.head_pos            = JointState()
        self.head_pos.name       = ["head_pan","head_tilt"]
        self.head_pos.position   = [0.0,self.pos_tilt]
        self.cnt_loss   = 0
        
        ##  error
        self.sum_err_pan  = 0
        self.sum_err_tilt = 0
        self.last_error_x = 0
        self.last_error_y = 0
        
        ## loss taget
        self.loss_count = 0
        
        ## scenario
        self.scenario = ""
        self.update = True ####
        
        ##  Publisher
        self.head_pub           = rospy.Publisher("/robotis/head_control/set_joint_states", JointState, queue_size=1)
        self.walk_set_param_pub = rospy.Publisher('/robotis/walking/set_params', WalkingParam, queue_size=5)
        self.walk_command_pub   = rospy.Publisher('/robotis/walking/command', String, queue_size=1)
        self.pub_loss           = rospy.Publisher("/sprint/loss",String, queue_size = 1)
        
        
        ##  Subscriber
        rospy.Subscriber("/sprint/marker/position"  , ArucoData,	self.marker_pos_callback)
        rospy.Subscriber("/sprint/scenario"         , String,	self.scenario_callback)
        rospy.Subscriber("/robotis/status", StatusMsg, self.status_callback)
        
        while not self.update:
            rospy.Rate(self.freq).sleep()
        
        
        ## Load forward backward params
        rospy.loginfo("[Walking] Loading PARAMS")
        self.walk_params = WalkingParam()
        
        self.load_walk_ini()
        
        self.walk_set_param_pub.publish(self.walk_params)
    
    def status_callback(self,msg):
        if self.update == False and msg.module_name == 'SENSOR':
            self.update = True
    
    def load_walk_ini(self):
        ## Path and Open yaml_file
        CONFIG_PATH = '/home/robotis/joe_ws/src/sprint_walk/config/'
        file_name='param.yaml'
        yaml_file = open(CONFIG_PATH + file_name)
        yf = yaml.load(yaml_file,Loader=yaml.FullLoader)
        print(yf["x_offset"])
        # parse movement time
        # rospy.loginfo("[Walking] Loading PARAMS")   ####
        # self.walk_params = WalkingParam()           ####

        self.walk_params.init_x_offset = yf["x_offset"]
        self.walk_params.init_x_offset = 0.01
        self.walk_params.init_y_offset = 0.038 # yf["y_offset"]
        self.walk_params.init_z_offset = 0.035 #yf["z_offset"]
        self.walk_params.init_roll_offset = np.radians(yf["roll_offset"])
        self.walk_params.init_pitch_offset =  np.radians(yf["pitch_offset"])
        self.walk_params.init_yaw_offset = np.radians(4) #np.radians(yf["yaw_offset"])
        self.walk_params.hip_pitch_offset = np.radians(yf["hip_pitch_offset"])
        # time
        self.walk_params.period_time = yf["period_time"] * 0.001    # ms -> s
        self.walk_params.dsp_ratio = yf["dsp_ratio"]
        self.walk_params.step_fb_ratio = yf["step_forward_back_ratio"]
        # walking
        self.walk_params.x_move_amplitude = 0.035
        self.walk_params.y_move_amplitude = 0
        self.walk_params.z_move_amplitude = yf["foot_height"]
        self.walk_params.angle_move_amplitude = np.radians(1) #yaw move

        # balance
        self.walk_params.balance_hip_roll_gain = yf["balance_hip_roll_gain"]
        self.walk_params.balance_knee_gain = yf["balance_knee_gain"]
        self.walk_params.balance_ankle_roll_gain = yf["balance_ankle_roll_gain"]
        self.walk_params.balance_ankle_pitch_gain = yf["balance_ankle_pitch_gain"]
        self.walk_params.y_swap_amplitude = yf["swing_right_left"]
        self.walk_params.z_swap_amplitude = yf["swing_top_down"]
        self.walk_params.pelvis_offset = 0.06 #np.radians(yf["pelvis_offset"])
        self.walk_params.arm_swing_gain = yf["arm_swing_gain"]
        # gain
        self.walk_params.p_gain = yf["p_gain"]
        self.walk_params.i_gain = yf["i_gain"]
        self.walk_params.d_gain = yf["d_gain"]

        self.walk_params.move_aim_on = False
        self.walk_params.balance_enable = True
    

        rospy.loginfo("[Walking] Loading PARAMS FINISH ")   ####

    def scenario_callback(self,msg):
        self.scenario = msg.data
        # print('hi')
        # print(self.scenario)
        if self.scenario == "init":
            self.load_walk_ini()
        
    def marker_pos_callback(self, msg):
        #  Aruco center marker
        self.marker['x']    = msg.x
        self.marker['y']    = msg.y
        
        # Distance
        loss = msg.size
        if loss != -1:
            self.tracking = True
            self.marker['size'] = msg.size
        if loss == -1:
            self.tracking = False
            self.loss_count += 1
            
        if self.debug:
            rospy.loginfo('[Walking] Marker: {}'.format(self.marker))
            
    def head_limit(self,pos_pan,pos_tilt):    
        # pan limiter
        pos_pan = np.radians(pos_pan)
        if pos_pan <= self.pan['min'] :
            self.head["pan"] = self.pan['min']
        elif pos_pan >= self.pan['max'] :
            self.head["pan"] = self.pan['max']
        # print(pos_pan)
        
        # tilt limiter
        if pos_tilt <= self.tilt['min'] :
            pos_tilt = self.tilt['min']
        elif pos_tilt >= self.tilt['max'] :
            pos_tilt = self.tilt['max']
        
        self.head['pan']  = round(pos_pan,  4)
        self.head['tilt'] = round(pos_tilt, 4)
        self.head_pos.position = [0,self.head['tilt']]
        self.head_pub.publish(self.head_pos)
    
    def track_marker(self):
        dt = 1.0 / self.freq
        Kp_pan = 0.15
        Ki_pan = 0
        Kd_pan = 0.05
        Kp_tilt = 0.01
        Ki_tilt = 0
        Kd_tilt = 0.015
        
        if self.marker['x'] != -1 and self.marker['y'] != -1:
            # pan
            error_x = (self.frame_w/2) - self.marker['x']
            if self.marker["size"] < 170:
                error_x *= 8*self.marker["size"] / self.frame_w # 77.32  
                
            elif self.marker["size"] < 240:
                error_x *= 15*self.marker["size"] / self.frame_w # 77.32  
            else:
                error_x *= 35*self.marker["size"] / self.frame_w # 77.32
            error_x = (error_x * math.pi)/ 180
            error_x_diff = error_x - self.last_error_x

            P_pan                = self.last_error_x * Kp_pan
            self.sum_err_pan    += error_x * dt
            I_pan                = self.sum_err_pan * Ki_pan
            deriv_err_pan        = error_x_diff / dt
            D_pan                = deriv_err_pan * Kd_pan
            self.last_error_x    = error_x
            self.head["pan"]     += (P_pan + I_pan + D_pan)
            
            # tilt
            error_y = (self.frame_h/2 +50 ) - self.marker['y']
            error_y *=  61.93 / self.frame_h
            error_y = (error_y * math.pi) /180
            error_y_diff = error_y - self.last_error_y
            
            P_tilt               = self.last_error_y * Kp_tilt
            self.sum_err_tilt   += error_y * dt
            I_tilt               = self.sum_err_tilt * Ki_tilt
            deriv_err_tilt       = error_y_diff / dt
            D_tilt               = deriv_err_tilt * Kd_tilt
            self.last_error_y    = error_y
            self.head["tilt"]    += (P_tilt + I_tilt + D_tilt)
            
            self.head_limit(self.head["pan"],self.head["tilt"]) 
            # print(self.head["pan"])
        # else:
        #     self.head["pan"] = 0
        # print(self.head["pan"])
    
    def run(self):
        rospy.loginfo("[Walking] START tracking")
        while not rospy.is_shutdown():
            ## Kp adjust
            Kp_fwd = 1.5
            Kp_bwd = -1.5
            
            error_body = self.head['pan']
            max_walk_a = np.radians(4)
            # print(self.scenari)
            
            if self.scenario == "forward":
                self.walk_params.x_move_amplitude = 0.035
                self.walk_params.y_move_amplitude = 0
                
                body_a = error_body * Kp_fwd
                if body_a <= -max_walk_a:
                    body_a = -max_walk_a
                elif body_a >= max_walk_a:
                    body_a = max_walk_a
                # body_a =0
                if self.tracking:
                    self.loss_count = 0
                # elif self.loss_count > 750:
                #     print(self.loss_count,body_a)
                #     body_a = body_a*2.5
                #     self.head["tilt"] = -np.radians(5)
                    
                elif self.loss_count > 250:
                    print(self.loss_count,body_a)
                    if self.marker["size"] < 180 and self.marker["size"] != -1:
                        self.head["tilt"] = -np.radians(50)
                        
                        self.walk_params.x_move_amplitude = 0.01
                        if body_a > 0:
                            self.walk_params.y_move_amplitude = 0.008
                        elif body_a < 0:
                            self.walk_params.y_move_amplitude = -0.008
                    if self.marker["size"] < 120 and self.loss_count > 250 and self.marker["size"] != -1 :
                        self.scenario = "backward"
                        self.walk_command_pub.publish('stop')
                        self.pub_loss.publish("back")
                        sleep(2)
                        # self.head["pan"] = -self.head["pan"]
                        self.walk_command_pub.publish('start')
                        
                    if self.loss_count > 750:
                        body_a = body_a*2.5
                        self.head["tilt"] = -np.radians(50)
                    if self.marker["size"] > 180:
                        self.head["tilt"] = -np.radians(10)
                        
                body_a=0
                    # rospy.loginfo("[WALKING] ADJUSTING !")
                print(body_a)
                self.walk_params.angle_move_amplitude = body_a #- np.radians(3) ###############
                # print(body_a)
                self.walk_set_param_pub.publish(self.walk_params)
                
                if self.debug:
                    rospy.loginfo("[Walking] FORWARD_ADJUSTING")

            elif self.scenario == "backward":
                self.walk_params.x_move_amplitude = -0.025
                self.walk_params.init_x_offset = -0.010 #yf["x_offset"]
                self.walk_params.z_move_amplitude = 0.060 # yf["foot_height"]
                self.walk_params.hip_pitch_offset = np.radians(7) #(yf["hip_pitch_offset"])
                
                self.walk_params.balance_hip_roll_gain = 0.30 #yf["balance_hip_roll_gain"]
                self.walk_params.balance_knee_gain = 0.30 #yf["balance_knee_gain"]
                self.walk_params.balance_ankle_roll_gain = 0.7 #yf["balance_ankle_roll_gain"]
                self.walk_params.balance_ankle_pitch_gain = 0.9 #yf["balance_ankle_pitch_gain"]
                # self.walk_params.y_swap_amplitude = 0.028 #yf["swing_right_left"]
                # self.walk_params.z_swap_amplitude = 0.006 #yf["swing_top_down"]
                # self.walk_params.pelvis_offset = 0.06 # np.radians(yf["pelvis_offset"]) #np.radians(0.5)
                self.walk_params.arm_swing_gain = 0.20 #yf["arm_swing_gain"]
                
                # self.walk_params.init_yaw_offset = 0 #np.radians(yf["yaw_offset"])
                
                body_a = error_body * Kp_bwd
                if body_a <= -max_walk_a:
                    body_a = -max_walk_a
                elif body_a >= max_walk_a:
                    body_a = max_walk_a 
                # self.walk_params.z_move_amplitude = 0.052 #yfDEGREE2RADIAN["foot_height"]
                # if body_a > 0:
                #     self.walk_params.y_move_amplitude = 0.01
                # elif body_a <0 :
                #     self.walk_params.y_move_amplitude = -0.01
                # else:
                #     self.walk_params.y_move_amplitude = 0.00
                if self.tracking:
                    self.loss_count = 0
                elif self.loss_count > 2500:
                    body_a = abs(body_a) * 2
                    print(self.loss_count,body_a)
                
                elif self.loss_count > 1800:
                    body_a =  -body_a * 1.5
                    print(self.loss_count,body_a)
                    
                elif self.loss_count > 750:
                    self.head["tilt"] = 0
                    # body_a = body_a*2.5    
                    print(self.loss_count,body_a)
                    
                self.walk_params.angle_move_amplitude = body_a - np.radians(3)  ###############
                # print(body_a)
                self.walk_set_param_pub.publish(self.walk_params)
                
                if self.debug:
                    rospy.loginfo("[Walking] BACKWARD_ADJUSTING")
            
            self.track_marker()
            rospy.Rate(self.freq).sleep()
                
if __name__ == "__main__":
    walking = Walking()
    walking.run()


