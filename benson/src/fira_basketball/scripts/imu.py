#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import numpy as np

def imu_callback(data):
    # print(data)
    # Extract the orientation quaternion from the IMU message
    angular_velocity = data.angular_velocity
    orientation = data.orientation
    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    # print(orientation)
    yaw2 = np.arctan2(2 * (orientation.w * orientation.z + orientation.x * orientation.y), 1 - 2 * (orientation.y ** 2 + orientation.z ** 2))
    # print(orientation.x, orientation.y, orientation.z, orientation.w, yaw2)
    # print(angular_velocity.z)

    # Convert quaternion to Euler angles
    roll, pitch, yaw = euler_from_quaternion(orientation_list)
    # return angular_velocity.z

    # Print the Euler angles
    rospy.loginfo("Roll: {:.2f} degrees Pitch: {:.2f} degrees Yaw: {:.2f} degrees".format(np.degrees(roll), np.degrees(pitch), np.degrees(yaw)))
    # rospy.loginfo()

def imu_subscriber():
    rospy.init_node('imu_subscriber', anonymous=True)
    rospy.Subscriber("/robotis/open_cr/imu", Imu, imu_callback)
    rospy.spin()

if __name__ == '__main__':
    imu_subscriber()
