#!/usr/bin/env python
# coding: utf-8
from time import sleep
import rospy
from moveit_commander.move_group import MoveGroupCommander

if __name__ == '__main__':

    rospy.init_node("dofbot_set_move")

    dofbot = MoveGroupCommander("dofbot")

    dofbot.allow_replanning(True)
    dofbot.set_planning_time(5)

    dofbot.set_num_planning_attempts(10)

    dofbot.set_goal_position_tolerance(0.01)

    dofbot.set_goal_orientation_tolerance(0.01)

    dofbot.set_goal_tolerance(0.01)

    dofbot.set_max_velocity_scaling_factor(1.0)

    dofbot.set_max_acceleration_scaling_factor(1.0)
    while (1):

       # dofbot.set_random_target()

        #dofbot.go()
        #sleep(0.5)

        dofbot.set_named_target("up")
        dofbot.go()
        sleep(0.5)

        dofbot.set_named_target("down")
        dofbot.go()
        sleep(0.5)
