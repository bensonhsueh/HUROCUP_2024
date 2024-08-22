#include<ros/ros.h>
#include<std_msgs/String.h>
#include<std_msgs/Int32.h>
#define _sleep(x) ros::Duration(x).sleep() 

std_msgs::String command_data;
std_msgs::Int32 command;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "benson_robot_begin");
    ros::NodeHandle hd;
    ros::Publisher pub_for_srart = hd.advertise<std_msgs::String>("/robotis/enable_ctrl_module",100);
    ros::Publisher pub_for_walk = hd.advertise<std_msgs::String>("/robotis/walking/command",100);
    ros::Publisher pub_for_init = hd.advertise<std_msgs::String>("/robotis/base/ini_pose",100);
    ros::Publisher pub_for_action = hd.advertise<std_msgs::Int32>("/robotis/action/page_num",100);
    ROS_INFO("HI");
    
    _sleep(1);
    
    command_data.data = "ini_pose";pub_for_init.publish(command_data);
    _sleep(5);
    
    command_data.data = "action_module";pub_for_srart.publish(command_data);
    _sleep(3);
    
    command.data = 2;pub_for_action.publish(command);
    _sleep(2);
    
    
    ROS_INFO("BYE");

}
