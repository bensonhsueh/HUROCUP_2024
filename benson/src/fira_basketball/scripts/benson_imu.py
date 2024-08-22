#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu

# y_linear = 0.0
z_gyro = 0.0
z_gyro_offset = 0.0
# y_linear_offset = 0.0
z_gyro_offset_for_caculate = 0.0
# y_linear_offset_for_caculate = 0.0
def imu_get_yaw_by_integral(data):
    global z_gyro
    global z_gyro_offset
    # global y_linear
    # global y_linear_offset

    angular_velocity = data.angular_velocity
    # linear_acceleration = data.linear_acceleration

    # print(data.linear_acceleration)
    z_gyro = z_gyro - angular_velocity.z/2 + z_gyro_offset_for_caculate
    # y_linear = y_linear - linear_acceleration.y + y_linear_offset_for_caculate
    # z_gyro = angular_velocity.z/2
    # print("z_gyro_offset_for_caculate:",z_gyro_offset_for_caculate)
    z_gyro_offset = angular_velocity.z/2
    # y_linear_offset = linear_acceleration.y


rospy.Subscriber("/robotis/open_cr/imu", Imu, imu_get_yaw_by_integral, queue_size=1)


# rospy.Subscriber("/video_source/raw", Image, vision.read)
rospy.init_node("benson_imu")



tickrate = 60
rate = rospy.Rate(tickrate)

rospy.sleep(1)
# y_linear_offset_for_caculate = y_linear_offset
z_gyro_offset_for_caculate = z_gyro_offset
z_gyro = 0.0
# y_linear = 0.0

# first_look_direction = ''



# STEP_LEVEL = 0

while not rospy.is_shutdown():
    print(z_gyro)
    # print(y_linear)



    rate.sleep()
        