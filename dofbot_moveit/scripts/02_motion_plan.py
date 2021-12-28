#!/usr/bin/env python
# coding: utf-8
import rospy
from Arm_Lib import Arm_Device
from math import pi
from time import sleep
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import MoveGroupCommander,PlanningSceneInterface
from tf.transformations import quaternion_from_euler


DE2RA = pi / 180

if __name__ == '__main__':

    rospy.init_node("dofbot_motion_plan_py")

    dofbot = MoveGroupCommander("dofbot")

    Arm = Arm_Device()
    dofbot.set_named_target("up")
    dofbot.go()
    sleep(2)
    Arm.Arm_serial_servo_write(6,60,200)
    sleep(2)

    dofbot.allow_replanning(True)
    dofbot.set_planning_time(5)

    dofbot.set_num_planning_attempts(10)

    dofbot.set_goal_position_tolerance(0.01)

    dofbot.set_goal_orientation_tolerance(0.01)

    dofbot.set_goal_tolerance(0.01)

    dofbot.set_max_velocity_scaling_factor(1.0)

    dofbot.set_max_acceleration_scaling_factor(1.0)
    #brown spot
    j1 = (88 - 90) * DE2RA
    j2 = (54 - 90) * DE2RA
    j3 = (51 - 90) * DE2RA
    j4 = (13 - 90) * DE2RA
    j5 = (90 - 90) * DE2RA
    target_joints1 = [j1, j2, j3, j4, j5]
    #red spot
    j1 = (115 - 90) * DE2RA
    j2 = (28 - 90) * DE2RA
    j3 = (75 - 90) * DE2RA
    j4 = (33 - 90) * DE2RA
    j5 = (90 - 90) * DE2RA
    target_joints2 = [j1, j2, j3, j4, j5]
    #target_joints2 = [0.47124,-1.134464,-0.47888,-0.59341,0]
    #target_joints2 = [90.0,53.0,33.0,36,270.0]
    tool_size = [0.03,0.03,0.03]
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
#    scene.attach_box(end_effector_link. 'tool',p,tool_size)

    while 1:
        dofbot.set_joint_value_target(target_joints1)
        #Arm.Arm_serial_servo_write(6,135,400)
        dofbot.go()
        sleep(2)
        print("brown finish")
        Arm.Arm_serial_servo_write(6,135,200)
        print("grip sent")
        sleep(2)
        print("grip finish")
        
        dofbot.set_named_target("up")
        dofbot.go()
        sleep(2)

        dofbot.set_joint_value_target(target_joints2)
        dofbot.go()
        Arm.Arm_serial_servo_write(6,60,200)
        sleep(2)

        dofbot.set_named_target("up")
        dofbot.go()
        sleep(2)
    #pos = Pose()

    #pos.position.x = 0.0
    #pos.position.y = 0.0597016
    #pos.position.z = 0.168051

    #roll = -140.0
    #pitch = 0.0
    #yaw = 0.0

    #q = quaternion_from_euler(roll * DE2RA, pitch * DE2RA, yaw * DE2RA)
    # pos.orientation.x = 0.940132
    # pos.orientation.y = -0.000217502
    # pos.orientation.z = 0.000375234
    # pos.orientation.w = -0.340811
    #pos.orientation.x = q[0]
    #pos.orientation.y = q[1]
    #pos.orientation.z = q[2]
    #pos.orientation.w = q[3]
    
    #dofbot.set_pose_target(pos)

    #for i in range(5):

    #    plan = dofbot.plan()
    #    if len(plan.joint_trajectory.points) != 0:
    #        print ("plan success")

     #       dofbot.execute(plan)
     #       break
     #   else:
     #       print ("plan error")
