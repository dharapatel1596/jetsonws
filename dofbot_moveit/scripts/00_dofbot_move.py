#!/usr/bin/env python
# coding: utf-8

import rospy
import Arm_Lib
from time import sleep
from math import pi
from sensor_msgs.msg import JointState


RA2DE = 180 / pi
DE2RA = pi / 180

def topic(msg):

    if not isinstance(msg, JointState): return

    joints = [0.0, 0.0, 0.0, 0.0, 0.0]

    for i in range(5):
        joints[i] = (msg.position[i] * RA2DE) + 90
        sbus.Arm_serial_servo_write(i+1, joints[i], 200)

    #joints.append(sbus.Arm_serial_servo_read(6))
    #print(joints)
    #sbus.Arm_serial_servo_write6_array(joints, 100)
        #sbus.Arm_serial_servo_write(i,joints[i],2000)

if __name__ == '__main__':

    sbus = Arm_Lib.Arm_Device()

    rospy.init_node("ros_dofbot")

    subscriber = rospy.Subscriber("/joint_states", JointState, topic)

    rate = rospy.Rate(2)

    rospy.spin()
