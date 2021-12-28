#!/usr/bin/env python
# coding: utf-8
import rospy
from math import pi
from time import sleep
from geometry_msgs.msg import Pose
from moveit_commander.move_group import MoveGroupCommander
from tf.transformations import quaternion_from_euler


DE2RA = pi / 180

if __name__ == '__main__':

    rospy.init_node("dofbot_motion_plan_py")

    dofbot = MoveGroupCommander("dofbot")

    dofbot.set_named_target("down")
    dofbot.go()
    sleep(0.5)

    dofbot.allow_replanning(True)
    dofbot.set_planning_time(5)

    dofbot.set_num_planning_attempts(10)

    dofbot.set_goal_position_tolerance(0.01)

    dofbot.set_goal_orientation_tolerance(0.01)

    dofbot.set_goal_tolerance(0.01)

    dofbot.set_max_velocity_scaling_factor(1.0)

    dofbot.set_max_acceleration_scaling_factor(1.0)

    pos = Pose()

    pos.position.x = 0.0
    pos.position.y = 0.0597016
    pos.position.z = 0.168051

    roll = -140.0
    pitch = 0.0
    yaw = 0.0

    q = quaternion_from_euler(roll * DE2RA, pitch * DE2RA, yaw * DE2RA)
    # pos.orientation.x = 0.940132
    # pos.orientation.y = -0.000217502
    # pos.orientation.z = 0.000375234
    # pos.orientation.w = -0.340811
    pos.orientation.x = q[0]
    pos.orientation.y = q[1]
    pos.orientation.z = q[2]
    pos.orientation.w = q[3]

    dofbot.set_pose_target(pos)

    for i in range(5):

        plan = dofbot.plan()
        if len(plan.joint_trajectory.points) != 0:
            print ("plan success")

            dofbot.execute(plan)
            break
        else:
            print ("plan error")
