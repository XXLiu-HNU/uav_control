"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""

#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

from mavros_msgs.msg import ActuatorControl

from mavros_msgs.msg import AttitudeTarget
current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg


if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    actuator_control = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=10)
    pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    mag = math.sqrt(0.08 * 0.08 + 1)
    attitude_wanted = AttitudeTarget()
    attitude_wanted.orientation.x = 0
    attitude_wanted.orientation.y = 0
    attitude_wanted.orientation.z = 0.5
    attitude_wanted.orientation.w = 0.086

    attitude_wanted.thrust = 0.8
    
    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        pub.publish(attitude_wanted)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        """
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()
        """
        pub.publish(attitude_wanted)

        rate.sleep()