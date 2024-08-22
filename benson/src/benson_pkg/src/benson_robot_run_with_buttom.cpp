#include<ros/ros.h>
#include<std_msgs/String.h>
#define _sleep(x) ros::Duration(x).sleep() 
bool ifstart=0;
void move(const std_msgs::String& msg)
{
    if(msg.data == "start")
    {
        ifstart = (-1)*ifstart + 1;
    }
    ROS_INFO("%s",msg.data.c_str());
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_action");
    ros::NodeHandle hd;
    ROS_INFO("HI");
    ros::Publisher pub_for_srart = hd.advertise<std_msgs::String>("/robotis/enable_ctrl_module",100);
    ros::Publisher pub_for_walk = hd.advertise<std_msgs::String>("/robotis/walking/command",100);
    ros::Publisher pub_for_init = hd.advertise<std_msgs::String>("/robotis/base/ini_pose",100);
    std_msgs::String command_data;
    _sleep(1);
    
    command_data.data = "ini_pose";pub_for_init.publish(command_data);_sleep(5);
    command_data.data = "walking_module";pub_for_srart.publish(command_data);_sleep(3);
    
    ros::Subscriber sub_for_but = hd.subscribe("/robotis/open_cr/button",1000,move);
    while(ros::ok())
    {
        if(ifstart){command_data.data = "start";pub_for_walk.publish(command_data);}
        else {command_data.data = "stop";pub_for_walk.publish(command_data);}
        ros::spinOnce();
        _sleep(0.5);
    }
    ROS_INFO("BYE");
}
