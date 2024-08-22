#!/usr/bin/python3

import rospy
from std_msgs.msg import Bool, Float32, Float32MultiArray

import os
import sys, tty, termios

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

from dynamixel_sdk import *
# from dynamixel_sdk import *                    # Uses Dynamixel SDK library
from dynamixel_sdk.port_handler import PortHandler
from dynamixel_sdk.packet_handler import PacketHandler
from dynamixel_sdk.robotis_def import  *
from dynamixel_sdk.group_sync_read import GroupSyncRead
from dynamixel_sdk.group_sync_write import GroupSyncWrite
from dynamixel_sdk.robotis_def import DXL_LOWORD,DXL_HIWORD,DXL_LOBYTE,DXL_HIBYTE
# Control table address
ADDR_PRO_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 30
ADDR_PRO_PRESENT_POSITION   = 36

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 2
LEN_PRO_PRESENT_POSITION    = 2

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting

RGRIPPER_ID                     = 21                 # Dynamixel#1 ID : 21
LGRIPPER_ID                     = 22                 # Dynamixel#1 ID : 22

BAUDRATE                    = 1000000        # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB1'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 5                # Dynamixel moving status threshold

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

# Initialize GroupSyncRead instace for Present Position
# groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()


def torque(enable):
    # Enable Dynamixel#21 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, RGRIPPER_ID, ADDR_PRO_TORQUE_ENABLE, enable)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        rospy.loginfo('Torque for Dynamixel #%d: %d' % (LGRIPPER_ID, enable))

    # Enable Dynamixel#22 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, LGRIPPER_ID, ADDR_PRO_TORQUE_ENABLE, enable)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        rospy.loginfo('Torque for Dynamixel #%d: %d' % (RGRIPPER_ID, enable))


# Add parameter storage for Dynamixel#1 present position value
# dxl_addparam_result = groupSyncRead.addParam(RGRIPPER_ID)
# if dxl_addparam_result != True:
#     print("[ID:%03d] groupSyncRead addparam failed" % RGRIPPER_ID)
#     quit()

# Add parameter storage for Dynamixel#2 present position value
# dxl_addparam_result = groupSyncRead.addParam(LGRIPPER_ID)
# if dxl_addparam_result != True:
#     print("[ID:%03d] groupSyncRead addparam failed" % LGRIPPER_ID)
#     quit()

def gripper_torque_callback(msg):
    if msg.data == True:
        torque(1)
    elif msg.data == False:
        torque(0)

def left_pos_callback(msg):
    val = msg.data

    if val > 100.0:
        val = 100.0
    elif val < 0.0:
        val = 0.0

    l_goal = int((2442-752)*val/100+752)
    print(l_goal)
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(l_goal)), DXL_HIBYTE(DXL_LOWORD(l_goal))]
    # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(LGRIPPER_ID, param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % LGRIPPER_ID)
        quit()

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

def right_pos_callback(msg):
    val = msg.data

    if val > 100.0:
        val = 100.0
    elif val < 0.0:
        val = 0.0
    
    r_goal = int((1638-3418)*val/100+3418)

    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(r_goal)), DXL_HIBYTE(DXL_LOWORD(r_goal))]

    # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(RGRIPPER_ID, param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % RGRIPPER_ID)
        quit()

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

def both_pos_callback(msg):
    left_val = msg.data[0]
    right_val = msg.data[1]

    r_goal = int((1638-3418)*val/100+3418)
    l_goal = int((2442-752)*val/100+752)

    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(r_goal)), DXL_HIBYTE(DXL_LOWORD(r_goal))]

    # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(RGRIPPER_ID, param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % RGRIPPER_ID)
        quit()

    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(l_goal)), DXL_HIBYTE(DXL_LOWORD(l_goal))]

    # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(LGRIPPER_ID, param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % LGRIPPER_ID)
        quit()

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()
 
    
rospy.init_node('gripper')
rospy.loginfo('Gripper node open.')

torque(1)
rospy.Subscriber("/grippers/torque", Bool, gripper_torque_callback)
rospy.Subscriber("/grippers/left_pos", Float32, left_pos_callback)
rospy.Subscriber("/grippers/right_pos", Float32, right_pos_callback)
rospy.Subscriber("/grippers/both_pos", Float32MultiArray, both_pos_callback)

rospy.spin()
