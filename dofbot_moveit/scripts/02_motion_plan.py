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
import sqlite3
from sqlite3 import Error


DE2RA = pi / 180

def create_connection(db_file):
    """ create a database connection to the SQLite database
        specified by the db_file
    :param db_file: database file
    :return: Connection object or None
    """
    conn = None
    try:
        conn = sqlite3.connect(db_file)
    except Error as e:
        print(e)
    return conn

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
    database = r"/home/jetson/dofbotDB"
    conn = create_connection(database)
    target_joints1 = [0.0, 0.0, 0.0, 0.0, 0.0]
    target_joints2 = [0.0, 0.0, 0.0, 0.0, 0.0]
    with conn:
        print("connection received\n")
	cur = conn.cursor()
	cur.execute("select * from SpotPosition  where spotname='brown'")
	raw = cur.fetchall()
    
        (joint1, joint2, joint3, joint4, joint5, spot)= tuple(raw[0])
        target_joints1[0] = (joint1 - 90) * DE2RA
        target_joints1[1] = (joint2 - 90) * DE2RA
        target_joints1[2] = (joint3 - 90) * DE2RA
        target_joints1[3] = (joint4 - 90) * DE2RA
        target_joints1[4] = (joint5 - 90) * DE2RA
        # for joint2
        cur.execute("select * from SpotPosition  where spotname='red'")
        raw = cur.fetchall()
    
        (joint1, joint2, joint3, joint4, joint5, spot) = tuple(raw[0])
        target_joints2[0] = (joint1 - 90) * DE2RA
        target_joints2[1] = (joint2 - 90) * DE2RA
        target_joints2[2] = (joint3 - 90) * DE2RA
        target_joints2[3] = (joint4 - 90) * DE2RA
        target_joints2[4] = (joint5 - 90) * DE2RA
    
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

    while 1:
        dofbot.set_joint_value_target(target_joints1)
        dofbot.go()
        sleep(0.5)
        Arm.Arm_serial_servo_write(6,135,100)
        sleep(0.5)

        dofbot.set_named_target("top")
        dofbot.go()
        sleep(0.5)

        dofbot.set_joint_value_target(target_joints2)
        dofbot.go()
        #sleep(0.5)
        Arm.Arm_serial_servo_write(6,60,100)
        #sleep(0.5)

        dofbot.set_named_target("top")
        dofbot.go()
        sleep(0.5)

