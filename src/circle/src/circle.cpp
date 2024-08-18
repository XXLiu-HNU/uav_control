/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */
 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
 
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
 
    //uav0
    ros::NodeHandle nh0;
    ros::Subscriber state_sub0 = nh0.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub0 = nh0.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client0 = nh0.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client0 = nh0.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
 
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
 
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    // x_num, y_num  refer to the desired uav input position
    //all the uavs have the same height : z
 
    float x0 = 0.0, y0 = 0.0;
    
    float z = 2.0;
    float w = 0.0;
    const float pi = 3.1415926;
    geometry_msgs::PoseStamped pose0;

    pose0.pose.position.x = x0;
    pose0.pose.position.y = y0;
    pose0.pose.position.z = z;
    
 
    //uavs have the initial position offset because of the .launch config file
    float x0_offset = -3;
    float x1_offset = -1;
    float x2_offset = 1;
    float x3_offset = 3;
 
    //define uavs' coordinates in the formation coordinate system
    float x0_f = -0.5, y0_f = 0.5;
    
    float xx, yy;
 
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub0.publish(pose0);
        ros::spinOnce();
        rate.sleep();
    }
 
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
 
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
 
    ros::Time last_request = ros::Time::now();
 
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
                
            if( set_mode_client0.call(offb_set_mode) ){
                ROS_INFO("Offboard enabled : 4");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client0.call(arm_cmd) ){
                    ROS_INFO("Vehicle armed : 4");
                    
                }
                last_request = ros::Time::now();
            }
        }
 
        local_pos_pub0.publish(pose0);
 
        w = w + 2*pi/(30/(1/20.0));
        if(w > 2*pi){
            w = w - 2*pi;
        }
        xx = 4.75*cos(w);
        yy = 4.75*sin(w);
        
        x0 = x0_f*cos(w) - y0_f*sin(w) + xx - x0_offset;
        y0 = y0_f*cos(w) + x0_f*sin(w) + yy;
        pose0.pose.position.x = x0;
        pose0.pose.position.y = y0;
 

        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}
 