#! /usr/bin/env python
import cv2
import os
import random
import rospy
from op3_ros_utils import *
from sensor_msgs.msg import Imu

taking_pic = False
taking_pic = True


z_gyro = 0.0
z_gyro_offset = 0.0
z_gyro_offset_for_caculate = 0.0
def imu_get_yaw_by_integral(data):
    global z_gyro
    global z_gyro_offset
    angular_velocity = data.angular_velocity
    z_gyro = z_gyro - angular_velocity.z/2 + z_gyro_offset_for_caculate
    z_gyro_offset = angular_velocity.z/2
    if(z_gyro > 180):
        z_gyro = z_gyro - 360
    if(z_gyro < -180):
        z_gyro = z_gyro + 360


rospy.Subscriber("/robotis/open_cr/imu", Imu, imu_get_yaw_by_integral, queue_size=1)
robot = Robot()


# rospy.Subscriber("/video_source/raw", Image, vision.read)
rospy.init_node("benson_safe_pic_from_cam")




def get_next_image_id(folder):
    """Get the next available image ID in the folder"""
    files = [f for f in os.listdir(folder) if f.endswith('.jpg')]
    if files:
        ids = [int(os.path.splitext(f)[0]) for f in files]
        return max(ids) + 1
    else:
        return 1

def witch_folder():
    """Randomly select a folder"""
    direction = int(z_gyro)
    print(direction)
    # selected_folder = random.choice(folders)
    selected_folder = os.path.join(base_folder, str(direction))
    return selected_folder

# Set the range of folders
folder_range = range(-180, 181)
base_folder = 'test'
folders = [os.path.join(base_folder, str(i)) for i in folder_range]

# Create folders
if not os.path.exists(base_folder):
    os.makedirs(base_folder)
for folder in folders:
    if not os.path.exists(folder):
        os.makedirs(folder)


tickrate = 60
rate = rospy.Rate(tickrate)

rospy.sleep(1)
# y_linear_offset_for_caculate = y_linear_offset
z_gyro_offset_for_caculate = z_gyro_offset
z_gyro = 0.0
# Open the camera
cap = cv2.VideoCapture(0)
while not rospy.is_shutdown():
    if robot.buttonCheck("mode"):
        z_gyro = 0.0
        print("reset z_gyro to 0")
    if robot.buttonCheck("user"):
        z_gyro = 180.0
        print("reset z_gyro to 180")
    if robot.buttonCheck("start"):
        if taking_pic:
            taking_pic = False
        else:
            taking_pic = True
    if taking_pic:
        # Read each frame from the camera
        ret, frame = cap.read()
        if not ret:
            print("Cannot read frame")
            break

        # Randomly select a folder
        selected_folder = witch_folder()

        # Get the next available image ID
        next_image_id = get_next_image_id(selected_folder)

        # Set the file name
        file_name = os.path.join(selected_folder, "{}.jpg".format(next_image_id))

        # Save the image
        cv2.imwrite(file_name, frame)
        print("Saved image to {}".format(file_name))
        cv2.putText(frame, str(int(z_gyro)) , (300, 100), cv2.FONT_HERSHEY_SIMPLEX, 3, (255,255,255), 3)


        # Display the frame (optional)
        cv2.imshow('Camera Frame', frame)
        cv2.waitKey(1)
        rate.sleep()
