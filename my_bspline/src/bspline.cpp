#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/String.h>
#include "my_bspline/uniform_bspline.h"
#include <visualization_msgs/Marker.h>
using namespace Eigen;
using namespace std;

ros::Time start_time_;
double traj_duration_;
bool flag = true;
geometry_msgs::PoseStamped pose;
visualization_msgs::Marker traj_marker;
visualization_msgs::Marker pose_marker;

ros::Publisher local_pos_pub;
ros::Publisher traj_marker_pub;
ros::Publisher real_traj_pub;

ros::Subscriber real_traj_sub;

void initializeMarker()
{
    traj_marker.header.frame_id = "map";
    traj_marker.ns = "trajectory";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
    traj_marker.action = visualization_msgs::Marker::ADD;
    traj_marker.scale.x = 0.05;
    traj_marker.color.r = 1.0;
    traj_marker.color.g = 0.0;
    traj_marker.color.b = 0.0;
    traj_marker.color.a = 1.0;

    pose_marker.header.frame_id = "map";
    pose_marker.ns = "trajectory";
    pose_marker.id = 1;
    pose_marker.type = visualization_msgs::Marker::LINE_STRIP;
    pose_marker.action = visualization_msgs::Marker::ADD;
    pose_marker.scale.x = 0.05;
    pose_marker.color.r = 0.0;
    pose_marker.color.g = 1.0;
    pose_marker.color.b = 0.0;
    pose_marker.color.a = 1.0;
}

void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    geometry_msgs::Point p;
    p.x = msg->pose.position.x;
    p.y = msg->pose.position.y;
    p.z = msg->pose.position.z;
    pose_marker.points.push_back(p);
    real_traj_pub.publish(pose_marker);
}

void cmdCallback(const ros::TimerEvent &e)
{
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time_).toSec();
    // 创建一个 3 行 15 列的随机矩阵
    Eigen::MatrixXd points(3, 15);

    points << 5, 6, 7, 8, 1, 4, 5, 7, 9, 8, 2, 4, 14, 12, 5,
        5, 7, 8, 3, 4, 9, 7, 8, 1, 5, 4, 4, 7, 5, 3,
        5, 8, 4, 3, 9, 7, 1, 5, 8, 7, 6, 4, 1, 4, 7;

    int order = 2;
    double interval = 2;
    UniformBspline bspline_(points, order, interval);
    Eigen::VectorXd knots = bspline_.getKnot();
    traj_duration_ = bspline_.getTimeSum();
    // ROS_INFO("traj_duration_ = %f", traj_duration_);
    //  std::stringstream ss;
    //  ss << "Knots vector: ";
    //  for (int i = 0; i < knots.size(); ++i) {
    //      ss << knots(i) << " ";
    //  }
    //  ROS_INFO("%s", ss.str().c_str());
    // bspline_.setKnot(knots);

    if (1)
    {
        Eigen::Vector3d pos;
        pos = bspline_.evaluateDeBoorT(t_cur);
        pose.pose.position.x = pos(0);
        pose.pose.position.y = pos(1);
        pose.pose.position.z = pos(2);

        local_pos_pub.publish(pose);

        geometry_msgs::Point p;
        p.x = pos(0);
        p.y = pos(1);
        p.z = pos(2);
        traj_marker.points.push_back(p);

        traj_marker_pub.publish(traj_marker);
        // ROS_INFO("t_cur = %f", t_cur);
        // ROS_INFO("Pos: x= %f, y= %f, z= %f", pos(0), pos(1), pos(2));
        // vectorList.push_back(pos);
    }
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "Bspline_test");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);
    start_time_ = ros::Time::now();

    initializeMarker();

    ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.05), cmdCallback);

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 100);
    real_traj_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, positionCallback);

    real_traj_pub = nh.advertise<visualization_msgs::Marker>("real_trajectory_marker", 10);
    traj_marker_pub = nh.advertise<visualization_msgs::Marker>("trajectory_marker", 10);
    while (ros::ok())
    {

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
