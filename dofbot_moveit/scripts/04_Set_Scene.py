#!/usr/bin/env python
# -*- coding: utf-8 -*-
from math import pi

import rospy, sys
from time import sleep
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface

if __name__ == "__main__":

    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('Set_Scene')

    scene = PlanningSceneInterface()

    dofbot = MoveGroupCommander('dofbot')

    dofbot.allow_replanning(True)
    dofbot.set_planning_time(5)

    dofbot.set_num_planning_attempts(10)

    dofbot.set_goal_position_tolerance(0.01)

    dofbot.set_goal_orientation_tolerance(0.01)

    dofbot.set_goal_tolerance(0.01)

    dofbot.set_max_velocity_scaling_factor(1.0)

    dofbot.set_max_acceleration_scaling_factor(1.0)

    dofbot.set_named_target("up")
    dofbot.go()
    sleep(0.5)
    target_joints1 = [0, -1.18, -1.17, 0.77, 0.03]
    target_joints2 = [0, -1.21, 0.52, -0.89, 0.08]
    tool_size = [0.03, 0.03, 0.03]

    end_effector_link = dofbot.get_end_effector_link()

    p = PoseStamped()
    p.header.frame_id = end_effector_link
    p.pose.position.x = 0
    p.pose.position.y = 0
    p.pose.position.z = 0.10
    p.pose.orientation.x = 0
    p.pose.orientation.y = 0
    p.pose.orientation.z = 0
    p.pose.orientation.w = 1

    scene.attach_box(end_effector_link, 'tool', p, tool_size)
    while 1:
        dofbot.set_joint_value_target(target_joints1)
        dofbot.go()
        sleep(0.5)
        dofbot.set_joint_value_target(target_joints2)
        dofbot.go()
        sleep(0.5)
