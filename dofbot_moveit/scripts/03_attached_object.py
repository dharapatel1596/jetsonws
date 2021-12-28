#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
from time import sleep
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface

if __name__ == "__main__":

    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('dofbot_attached_object_py')

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

    table_ground = 0.2

    table_size = [0.7, 0.1, 0.02]

    table_pose = PoseStamped()
    table_pose.header.frame_id = 'base_link'
    table_pose.pose.position.x = 0
    table_pose.pose.position.y = 0.15
    table_pose.pose.position.z = table_ground + table_size[2] / 2.0
    table_pose.pose.orientation.w = 1.0
    scene.add_box('table', table_pose, table_size)
    rospy.sleep(2)
    dofbot.set_named_target("down")
    dofbot.go()
    sleep(0.5)
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

