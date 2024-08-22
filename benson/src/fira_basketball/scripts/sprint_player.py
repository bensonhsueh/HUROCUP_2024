#! /usr/bin/env python
import numpy as np
import math
import time
import rospy
import roslib
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Int32, Bool, Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

frame_w = 320#rospy.get_param("/usb_cam/image_width")
frame_h = 240#rospy.get_param("/usb_cam/image_height")

marker_x = -1
marker_y = -1
marker_size = -1
def marker_pos_callback(pos_msg):
	global marker_x, marker_y, marker_size
	marker_x = pos_msg.data[0]
	marker_y = pos_msg.data[1]
	marker_size = pos_msg.data[2]

# line_x = -1
# line_y = -1
# def line_pos_callback(pos_msg):
# 	global line_x, line_y
# 	line_x = pos_msg.data[0]
# 	# line_y = pos_msg.data[1]
# 	# print(line_x, line_y)

button = [0, 0]
def button_callback(pos_msg):
	global button
	button[0] = pos_msg.data[0]
	button[1] = pos_msg.data[1]

magneto = 0
def compass_callback(cmps_msg):
	global magneto
	magneto = cmps_msg.data

count_marker_loss  = 0
def marker_lost(threshold):
	global count_marker_loss
	if marker_x == -1 and marker_y == -1:		
		count_marker_loss += 1
		if count_marker_loss >= threshold :
			return True
	else :
		count_marker_loss = 0
		return False

# count_line_loss  = 0
# def line_lost(threshold):
# 	global count_line_loss
# 	if line_x == -1 and line_y == -1:		
# 		count_line_loss += 1
# 		if count_line_loss >= threshold :
# 			return True
# 	else :
# 		count_line_loss = 0
# 		return False

def head_move(head_pan, head_tilt):
	global pos_pan, pos_tilt
	pos_pan = head_pan
	pos_tilt = head_tilt
	head_pos = Float32MultiArray()
	head_pos.data = [pos_pan, pos_tilt]
	head_pub.publish(head_pos)

def walk(x, y, a):
	velocity = Twist()
	velocity.linear.x = x
	velocity.linear.y = y
	velocity.linear.z = a
	motion_vel_pub.publish(velocity)

pos_pan = 0.0
pos_tilt = 0.0
pan_min = -1.0
pan_max = 1.0
tilt_min = -2.0
tilt_max = -0.1

def head_limit(pos_pan, pos_tilt):
	if pos_pan <= pan_min :		
		pos_pan = pan_min
	elif pos_pan >= pan_max :		
		pos_pan = pan_max
	if pos_tilt <= tilt_min :		
		pos_tilt = tilt_min
	elif pos_tilt >= tilt_max :	
		pos_tilt = tilt_max
	head_pos = Float32MultiArray()
	head_pos.data = [pos_pan, pos_tilt]
	return head_pos

pan_step = 0.0
tilt_step = 0.0
move_pan = True

def scan_marker(mode):
	global pos_pan, pos_tilt, pan_step, tilt_step, move_pan

	if pan_step > 0:		
		pan_step = rospy.get_param("/sprint_params/player/Pan_Step")
	else:				
		pan_step = rospy.get_param("/sprint_params/player/Pan_Step") * -1
	if tilt_step > 0:	
		tilt_step = rospy.get_param("/sprint_params/player/Tilt_Step") 
	else:				
		tilt_step = rospy.get_param("/sprint_params/player/Tilt_Step") * -1
	
	if mode == 0: # normal
		pos_pan += pan_step
		if pos_pan >= pan_max or pos_pan <= pan_min:
			pan_step *= -1
			pos_tilt += tilt_step
			if pos_tilt >= tilt_max or pos_tilt <= tilt_min:
				tilt_step *= -1 

	elif mode == 1: # only tilt
		pos_pan = 0.0
		pos_tilt += tilt_step
		if pos_tilt >= tilt_max or pos_tilt <= tilt_min:
			tilt_step *= -1 

	elif mode == 2: # rectangle
		if move_pan: 
			pos_pan += pan_step
			if pos_pan >= pan_max or pos_pan <= pan_min:
				pan_step *= -1
				move_pan = False
		else:
			pos_tilt += tilt_step
			if pos_tilt >= tilt_max or pos_tilt <= tilt_min:
				tilt_step *= -1
				move_pan = True

	elif mode == 3: # only pan
		pos_tilt = -1.0
		pos_pan += pan_step
		if pos_pan >= pan_max or pos_pan <= pan_min:
			pan_step *= -1 

	head_pos = head_limit(pos_pan, round(pos_tilt, 3))
	pos_pan, pos_tilt = head_pos.data
	head_pub.publish(head_pos)

sum_err_pan = 0
sum_err_tilt = 0
last_error_x = 0
last_error_y = 0
def head_track_marker():
	global pos_pan, pos_tilt, sum_err_pan, sum_err_tilt, last_error_x, last_error_y
	global freq
	dt = 1.0 / float(freq)
	KP_pan = rospy.get_param("/sprint_params/player/Pan_KP")
	KI_pan = rospy.get_param("/sprint_params/player/Pan_KI")
	KD_pan = rospy.get_param("/sprint_params/player/Pan_KD")
	KP_tilt = rospy.get_param("/sprint_params/player/Tilt_KP")
	KI_tilt = rospy.get_param("/sprint_params/player/Tilt_KI")
	KD_tilt = rospy.get_param("/sprint_params/player/Tilt_KD")
	if marker_x != -1 and marker_y != -1:	
		error_x = (frame_w/2) - marker_x
		error_x *= 77.32 / frame_w
		error_x = (error_x * math.pi)/ 180
		error_x_diff = error_x - last_error_x

		P_pan  = last_error_x * KP_pan
		sum_err_pan += error_x * dt
		I_pan = sum_err_pan * KI_pan
		deriv_err_pan = error_x_diff / dt
		D_pan = deriv_err_pan * KD_pan
		last_error_x = error_x
		pos_pan += (P_pan + I_pan + D_pan)*-1

		error_y = (frame_h/2) - marker_y
		error_y *= -1
		error_y *= 61.93 / frame_h
		error_y = (error_y * math.pi) /180
		error_y_diff = error_y - last_error_y

		P_tilt  = last_error_y * KP_tilt
		sum_err_tilt += error_y * dt
		I_tilt = sum_err_tilt * KI_tilt
		deriv_err_tilt = sum_err_tilt / dt
		D_tilt = deriv_err_tilt * KD_tilt
		last_error_y = error_y
		pos_tilt += (P_tilt + I_tilt + D_tilt)*-1

		head_pos = head_limit(pos_pan, round(pos_tilt, 2))
		#print("Marker Size: %d", marker_size)
		pos_pan, pos_tilt = head_pos.data		
		head_pub.publish(head_pos)

def body_track_marker(mode):
	global pos_pan, pos_tilt
	KP_body_forward = rospy.get_param("/sprint_params/player/Body_Forward_KP")
	KP_body_backward = rospy.get_param("/sprint_params/player/Body_Backward_KP")
	if marker_x != -1 and marker_y != -1:	 
		error_body_a = pos_pan - 0
	else:
		error_body_a = 0
	max_walk_a = 0.4
	if mode == 0:
		body_a = error_body_a * KP_body_forward
	elif mode == 1:
		body_a = error_body_a * KP_body_backward

	# if error_body_a >= -0.1 and error_body_a <= 0.1:
	# 	body_a = 0
	# else :			
	if body_a >= max_walk_a:
		body_a = max_walk_a
	elif body_a <= -max_walk_a:
		body_a = -max_walk_a
	body_a = round(body_a, 2)
	return body_a

# sum_err_pan = 0
# sum_err_tilt = 0
# last_error_x = 0
# last_error_y = 0
# def head_track_line():
# 	global pos_pan, pos_tilt, sum_err_pan, sum_err_tilt, last_error_x, last_error_y
# 	global freq
# 	dt = 1.0 / float(freq)
# 	KP_pan = rospy.get_param("/sprint_params/player/Pan_KP")
# 	KI_pan = rospy.get_param("/sprint_params/player/Pan_KI")
# 	KD_pan = rospy.get_param("/sprint_params/player/Pan_KD")
# 	KP_tilt = rospy.get_param("/sprint_params/player/Tilt_KP")
# 	KI_tilt = rospy.get_param("/sprint_params/player/Tilt_KI")
# 	KD_tilt = rospy.get_param("/sprint_params/player/Tilt_KD")
# 	if line_x != -1 and line_y != -1:	
# 		error_x = (frame_w/2) - line_x
# 		error_x *= 77.32 / frame_w
# 		error_x = (error_x * math.pi)/ 180
# 		error_x_diff = error_x - last_error_x

# 		P_pan  = last_error_x * KP_pan
# 		sum_err_pan += error_x * dt
# 		I_pan = sum_err_pan * KI_pan
# 		deriv_err_pan = error_x_diff / dt
# 		D_pan = deriv_err_pan * KD_pan
# 		last_error_x = error_x
# 		pos_pan += (P_pan + I_pan + D_pan)
# 		pos_tilt = -1.0

# 		head_pos = head_limit(pos_pan, round(pos_tilt, 2))
# 		# print("pospan: %f", pos_pan)
# 		pos_pan, pos_tilt = head_pos.data		
# 		head_pub.publish(head_pos)

def body_track_marker(mode):
	global pos_pan, pos_tilt
	KP_body_forward = rospy.get_param("/sprint_params/player/Body_Forward_KP")
	KP_body_backward = rospy.get_param("/sprint_params/player/Body_Backward_KP")
	if marker_x != -1 and marker_y != -1:	 
		error_body_a = pos_pan - 0
	else:
		error_body_a = 0
	max_walk_a = 0.4
	if mode == 0:
		body_a = error_body_a * KP_body_forward
	elif mode == 1:
		body_a = error_body_a * KP_body_backward

	# if error_body_a >= -0.1 and error_body_a <= 0.1:
	# 	body_a = 0
	# else :			
	if body_a >= max_walk_a:
		body_a = max_walk_a
	elif body_a <= -max_walk_a:
		body_a = -max_walk_a
	body_a = round(body_a, 2)
	return body_a

def kill_node():
	rospy.signal_shutdown("shutdown time.") 

def main():
	print("Sprint Player - Running")
	rospy.init_node("sprint_player")
	rospy.wait_for_service("/srv_controller")
	
	global head_pub, motion_vel_pub, motion_state_pub
	motion_vel_pub = rospy.Publisher("/motion/cmd_vel", Twist, queue_size=1)
	motion_state_pub = rospy.Publisher("/motion/state", String, queue_size=1)
	gripper_state_pub = rospy.Publisher("/gripper/state", Bool, queue_size=1)	
	head_pub = rospy.Publisher("/head/pos", Float32MultiArray, queue_size=1)
	marker_pos_sub = rospy.Subscriber("/sprint/marker/position", Int32MultiArray, marker_pos_callback)
	# line_pos_sub = rospy.Subscriber("/sprint/lines/position", Int32MultiArray, line_pos_callback)
	button_sub = rospy.Subscriber("/button/state", Int32MultiArray, button_callback)
	# compass_sub = rospy.Subscriber("/compass/value", Int32, compass_callback)

	print("Sprint Player - Running")
	time.sleep(0.3)
	motion_state_pub.publish("stand")
	global freq
	freq = 50
	rate = rospy.Rate(freq)
	state = "initial"
	play = True
	button_pressed = [0, 0]
	conf_stop = 0; conf_start = 0

	while not rospy.is_shutdown():
		if button[0] == 1:
			button_pressed[0] = 1
		else:
			if button_pressed[0] == 1:
				if play:
					motion_state_pub.publish("sit")
					print("Sit")
					play = False
				else:
					motion_state_pub.publish("stand")
					print("Stand")
					play = True
					state = "initial"
				button_pressed[0] = 0

		#///////////////////////////////////////////////////////////////////////
		#//////////////.............Role of execution............///////////////
		#///////////////////////////////////////////////////////////////////////
		if play :
			#print(state)
			if state == "initial":
				if marker_lost(20):
					scan_marker(1)
					# head_move(0.0, -1.3)
				else:
					head_track_marker()
					if button[1] == 1:
						motion_state_pub.publish("start")
						state = "walk_warmup"
			elif state == "walk_warmup":				 
				if conf_start > 30:
					conf_start = 0
					state = "forward"
				else:
					conf_start += 1	
			elif state == "forward":
				if marker_lost(20):
					scan_marker(1)
					walk(0.05, 0.0, 0.00)
				else:
					if marker_size < 45000:
						head_track_marker()
						shift = body_track_marker(0)
						walk(0.05, 0.0, shift)
					else:
						state = "backward"
			elif state == "backward":
				if marker_lost(20):
					scan_marker(1)
					walk(-0.4, 0.0, 0.0)
				else:
					head_track_marker()
					if marker_size > 1:						
						shift = body_track_marker(1)
						walk(-0.4, 0.0, shift)
						conf_stop = 0
					else:
						if conf_stop > 30:
							print("stop")
							motion_state_pub.publish("stop")
							state = "initial"
						else:
					 		conf_stop += 1

			elif state == "tune_head":
				if marker_lost(20):
					scan_marker(1)
				else:
					head_track_marker()
				print("%d, %d", marker_x, marker_y)
			
			elif state == "tune_body":
				if marker_lost(20):
					motion_state_pub.publish("stop")
					scan_marker(1)
				else:
					head_track_marker()
					shift = body_track_marker()
					walk(0.0, 0.0, shift)
				print("%d, %d", marker_x, marker_y)

			# elif state == "forward_line":
			# 	if line_lost(30):
			# 		# pos_pan = 0.0
			# 		# pos_tilt= -1.0
			# 		# head_pos = head_limit(pos_pan, round(pos_tilt, 2))
			# 		# pos_pan, pos_tilt = head_pos.data		
			# 		# head_pub.publish(head_pos)
			# 		scan_marker(3)
			# 		# walk(0.02, 0.0, 0.00)
			# 	else:
			# 		head_track_line()
			# 		# shift = body_track_line(0)
			# 		# walk(0.02, 0.0, shift)
		
		rate.sleep()
	print("Sprint Player - Shut Down")
	rospy.on_shutdown(kill_node)

if __name__ == "__main__":
	main()
