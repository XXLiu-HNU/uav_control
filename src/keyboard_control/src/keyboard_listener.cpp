#include <ros/ros.h>
#include <std_msgs/String.h>

void callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received keyboard input: %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("keyboard_input", 10, callback);
    ros::spin();
    return 0;
}
