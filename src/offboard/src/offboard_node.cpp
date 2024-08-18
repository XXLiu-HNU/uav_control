#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/String.h>

using namespace Eigen;
using namespace std;
//定义话题名称
ros::Publisher local_pos_pub;
ros::Subscriber key_sub;
// 定义目标位置和更新标志位
Eigen::Vector3d target_position;
bool position_updated  = false;
//定义键盘接受回调函数
void callback(const std_msgs::String::ConstPtr &msg)
{
    //打印接受数据
    ROS_INFO("Received keyboard input: %s", msg->data.c_str());
    position_updated  = true;
    // 根据接收到的字符执行不同操作
    if (msg->data == "w")
    {
        target_position = target_position + Eigen::Vector3d(1.0, 0, 0);
    }
    else if (msg->data == "s")
    {
        target_position = target_position + Eigen::Vector3d(-1, 0, 0);
    }
    else if (msg->data == "a")
    {
        target_position = target_position + Eigen::Vector3d(0, 1, 0);
    }
    else if (msg->data == "d")
    {
        target_position = target_position + Eigen::Vector3d(0, -1, 0);
    }
    else if (msg->data == "p")
    {
        target_position = target_position + Eigen::Vector3d(0, 0, 1);
    }
    else if (msg->data == "l")
    {
        target_position = target_position + Eigen::Vector3d(0, 0, -1);
    }
    else
    {
        ROS_INFO("input must be 'w,a,s,d,p,l'");
        position_updated  = false;
    }
    
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Subscriber key_sub = nh.subscribe("keyboard_input", 10, callback);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    target_position << 0.0, 0.0, 1.0;
    while (ros::ok())
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = target_position[0];
        pose.pose.position.y = target_position[1];
        pose.pose.position.z = target_position[2];
        if ( position_updated )
        {
            ROS_INFO("Target position %f,%f,%f", target_position[0], target_position[1], target_position[2]);
            position_updated = false;
        }

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
