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
#include "quadrotor_msgs/PositionCommand.h"
#include "/home/xingxun/ego-planner/devel/include/ego_planner/Bspline.h"

#include "nav_msgs/Odometry.h"

#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>

using namespace Eigen;
using namespace std;

ros::Time start_time_;
double traj_duration_;
bool receive_traj_ = false;
visualization_msgs::Marker traj_marker;
visualization_msgs::Marker pose_marker;
int traj_id_;
// for control
vector<UniformBspline> traj_;
quadrotor_msgs::PositionCommand cmd;
double last_yaw_, last_yaw_dot_;
double time_forward_;

ros::Publisher local_pos_pub;
ros::Publisher traj_marker_pub;
ros::Publisher real_traj_pub;
ros::Publisher pos_cmd_pub;
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

void bsplineCallback(ego_planner::BsplineConstPtr msg)
{
    // parse pos traj
    ROS_INFO("IN here");
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i)
    {
        knots(i) = msg->knots[i];
    }

    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
    {
        pos_pts(0, i) = msg->pos_pts[i].x;
        pos_pts(1, i) = msg->pos_pts[i].y;
        pos_pts(2, i) = msg->pos_pts[i].z;
    }

    UniformBspline pos_traj(pos_pts, msg->order, 0.1);
    pos_traj.setKnot(knots);

    // parse yaw traj

    // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
    // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
    //   yaw_pts(i, 0) = msg->yaw_pts[i];
    // }

    // UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

    start_time_ = msg->start_time;
    traj_id_ = msg->traj_id;

    traj_.clear();
    traj_.push_back(pos_traj);
    traj_.push_back(traj_[0].getDerivative());
    traj_.push_back(traj_[1].getDerivative());

    traj_duration_ = traj_[0].getTimeSum();

    receive_traj_ = true;
}
std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
    constexpr double PI = 3.1415926;
    constexpr double YAW_DOT_MAX_PER_SEC = PI;
    // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
    std::pair<double, double> yaw_yawdot(0, 0);
    double yaw = 0;
    double yawdot = 0;

    Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
    double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
    double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
    if (yaw_temp - last_yaw_ > PI)
    {
        if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
        {
            yaw = last_yaw_ - max_yaw_change;
            if (yaw < -PI)
                yaw += 2 * PI;

            yawdot = -YAW_DOT_MAX_PER_SEC;
        }
        else
        {
            yaw = yaw_temp;
            if (yaw - last_yaw_ > PI)
                yawdot = -YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }
    else if (yaw_temp - last_yaw_ < -PI)
    {
        if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
        {
            yaw = last_yaw_ + max_yaw_change;
            if (yaw > PI)
                yaw -= 2 * PI;

            yawdot = YAW_DOT_MAX_PER_SEC;
        }
        else
        {
            yaw = yaw_temp;
            if (yaw - last_yaw_ < -PI)
                yawdot = YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }
    else
    {
        if (yaw_temp - last_yaw_ < -max_yaw_change)
        {
            yaw = last_yaw_ - max_yaw_change;
            if (yaw < -PI)
                yaw += 2 * PI;

            yawdot = -YAW_DOT_MAX_PER_SEC;
        }
        else if (yaw_temp - last_yaw_ > max_yaw_change)
        {
            yaw = last_yaw_ + max_yaw_change;
            if (yaw > PI)
                yaw -= 2 * PI;

            yawdot = YAW_DOT_MAX_PER_SEC;
        }
        else
        {
            yaw = yaw_temp;
            if (yaw - last_yaw_ > PI)
                yawdot = -YAW_DOT_MAX_PER_SEC;
            else if (yaw - last_yaw_ < -PI)
                yawdot = YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }

    if (fabs(yaw - last_yaw_) <= max_yaw_change)
        yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
    yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;

    yaw_yawdot.first = yaw;
    yaw_yawdot.second = yawdot;

    return yaw_yawdot;
}
void cmdCallback(const ros::TimerEvent &e)
{
    /* no publishing before receive traj_ */
    if (!receive_traj_)
        return;

    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time_).toSec();

    Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
    std::pair<double, double> yaw_yawdot(0, 0);
    static ros::Time time_last = ros::Time::now();
    if (t_cur < traj_duration_ && t_cur >= 0.0)
    {
        pos = traj_[0].evaluateDeBoorT(t_cur);
        vel = traj_[1].evaluateDeBoorT(t_cur);
        acc = traj_[2].evaluateDeBoorT(t_cur);

        /*** calculate yaw ***/
        yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
        /*** calculate yaw ***/

        double tf = min(traj_duration_, t_cur + 2.0);
        pos_f = traj_[0].evaluateDeBoorT(tf);
    }
    else if (t_cur >= traj_duration_)
    {
        /* hover when finish traj_ */
        pos = traj_[0].evaluateDeBoorT(traj_duration_);
        vel.setZero();
        acc.setZero();

        yaw_yawdot.first = last_yaw_;
        yaw_yawdot.second = 0;

        pos_f = pos;
        return;
    }
    else
    {
        cout << "[Traj server]: invalid time." << endl;
    }
    time_last = time_now;
    cmd.header.stamp = time_now;
    cmd.header.frame_id = "world";
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = traj_id_;

    cmd.position.x = pos(0);
    cmd.position.y = pos(1);
    cmd.position.z = pos(2);

    cmd.velocity.x = vel(0);
    cmd.velocity.y = vel(1);
    cmd.velocity.z = vel(2);

    cmd.acceleration.x = acc(0);
    cmd.acceleration.y = acc(1);
    cmd.acceleration.z = acc(2);

    cmd.yaw = yaw_yawdot.first;
    cmd.yaw_dot = yaw_yawdot.second;

    last_yaw_ = cmd.yaw;

    pos_cmd_pub.publish(cmd);

    // vis target_position
    geometry_msgs::Point p;
    p.x = pos(0);
    p.y = pos(1);
    p.z = pos(2);
    traj_marker.points.push_back(p);
    traj_marker_pub.publish(traj_marker);
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "Bspline_test");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);
    start_time_ = ros::Time::now();

    initializeMarker();

    ros::Subscriber bspline_sub = nh.subscribe("planning/bspline", 10, bsplineCallback);
    nh.param("traj_server/time_forward", time_forward_, -1.0);
    ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.05), cmdCallback);

    pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

    // local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 100);
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
