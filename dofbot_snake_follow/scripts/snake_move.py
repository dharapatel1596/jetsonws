#!/usr/bin/env python
# coding: utf-8
import Arm_Lib
from time import sleep


class snake_move:
    def __init__(self):

        self.arm = Arm_Lib.Arm_Device()
        # Angle of servo when clamp close
        self.grap_joint = 135

    def arm_move(self, joints_target):
        '''
        move process
        '''
        # Rotate to horizontal
        self.arm.Arm_serial_servo_write(5, 90, 500)
        sleep(1)
        # Open clip 
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(1.5)
        # 夹紧夹爪
        self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        sleep(0.6)
        # 移动至对应位置
        self.arm.Arm_serial_servo_write6_array(joints_target, 1000)
        sleep(1.5)
        # 松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.5)
        # 回到初始位置
        joint_00 = [90, 135, 0, 45, 0, 180]
        self.arm.Arm_serial_servo_write6_array(joint_00, 500)
        sleep(1)

    def snake_run(self, name):
        '''
        DOFBOT move process
        '''
        if name == "red":
            # print("red")
            joints_target = [117, 19, 66, 56, 90, self.grap_joint]
            self.arm_move(joints_target)
        if name == "blue":
            # print("blue")
            joints_target = [44, 66, 20, 28, 90, self.grap_joint]
            self.arm_move(joints_target)
        if name == "green":
            # print("green")
            joints_target = [136, 66, 20, 29, 90, self.grap_joint]
            self.arm_move(joints_target)
        if name == "yellow":
            # print("yellow")
            joints_target = [65, 22, 64, 56, 90, self.grap_joint]
            self.arm_move(joints_target)
