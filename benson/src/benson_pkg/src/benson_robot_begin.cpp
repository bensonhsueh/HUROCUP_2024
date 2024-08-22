#include<ros/ros.h>
#include<std_msgs/String.h>
#define _sleep(x) ros::Duration(x).sleep() 

std_msgs::String command_data;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "benson_robot_begin");
    ros::NodeHandle hd;
    ros::Publisher pub_for_srart = hd.advertise<std_msgs::String>("/robotis/enable_ctrl_module",100);
    ros::Publisher pub_for_walk = hd.advertise<std_msgs::String>("/robotis/walking/command",100);
    ros::Publisher pub_for_init = hd.advertise<std_msgs::String>("/robotis/base/ini_pose",100);
    ROS_INFO("HI");
    
    _sleep(1);
    
    command_data.data = "ini_pose";pub_for_init.publish(command_data);
    _sleep(5);
    
    command_data.data = "walking_module";pub_for_srart.publish(command_data);
    _sleep(3);
    
    command_data.data = "start";pub_for_walk.publish(command_data);
    _sleep(2);
    
    command_data.data = "stop";pub_for_walk.publish(command_data);
    _sleep(2);
    
    command_data.data = "ini_pose";pub_for_init.publish(command_data);
    
    
    ROS_INFO("BYE");

}
