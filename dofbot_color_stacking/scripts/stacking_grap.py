#!/usr/bin/env python
# coding: utf-8
import Arm_Lib
from time import sleep


class stacking_grap:
    def __init__(self):

        self.move_status = True

        self.arm = Arm_Lib.Arm_Device()

        self.grap_joint = 140

    def move(self, joints, joints_down):

        joints_00 = [90, 80, 50, 50, 265, self.grap_joint]

        joints_up = [137, 80, 50, 50, 265, 30]

        self.arm.Arm_serial_servo_write6_array(joints_00, 1000)
        sleep(1)

        for i in range(5):
            self.arm.Arm_serial_servo_write(6, 180, 100)
            sleep(0.08)
            self.arm.Arm_serial_servo_write(6, 30, 100)
            sleep(0.08)
#         
#         self.arm.Arm_serial_servo_write(6, 0, 500)
#         sleep(0.5)

        self.arm.Arm_serial_servo_write6_array(joints, 1000)
        sleep(1)

        self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        sleep(0.5)

        self.arm.Arm_serial_servo_write6_array(joints_00, 1000)
        sleep(1)

        self.arm.Arm_serial_servo_write(1, joints_down[0], 1000)
        sleep(1)

        self.arm.Arm_serial_servo_write6_array(joints_down, 1000)
        sleep(1.5)

        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.5)

        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        sleep(1)

    def arm_run(self, move_num, joints):

        # a=[132, 50, 20, 60, 265, 100]
        # b=[132, 55, 38, 38, 265, 100]
        # c=[132, 60, 45, 30, 265, 100]
        # d=[132, 65, 55, 20, 265, 100]
        if move_num == '1' and self.move_status == True:

            self.move_status = False  
            # print(joints[0], joints[1], joints[2], joints[3], joints[4])

            joints = [joints[0], joints[1], joints[2], joints[3], 265, 30]

            joints_down = [137, 50, 20, 60, 265, self.grap_joint]

            self.move(joints, joints_down)

            self.move_status = True
        if move_num == '2' and self.move_status == True:
            self.move_status = False
            # print joints[0], joints[1], joints[2], joints[3], joints[4]
            joints = [joints[0], joints[1], joints[2], joints[3], 265, 30]
            joints_down = [137, 55, 38, 38, 265, self.grap_joint]
            self.move(joints, joints_down)
            self.move_status = True
        if move_num == '3' and self.move_status == True:
            self.move_status = False
            # print joints[0], joints[1], joints[2], joints[3], joints[4]
            joints = [joints[0], joints[1], joints[2], joints[3], 265, 30]
            joints_down = [137, 60, 45, 30, 265, self.grap_joint]
            self.move(joints, joints_down)
            self.move_status = True
        if move_num == '4' and self.move_status == True:
            self.move_status = False
            # print joints[0], joints[1], joints[2], joints[3], joints[4]
            joints = [joints[0], joints[1], joints[2], joints[3], 265, 30]
            joints_down = [137, 65, 55, 20, 265, self.grap_joint]
            self.move(joints, joints_down)
            self.move_status = True
