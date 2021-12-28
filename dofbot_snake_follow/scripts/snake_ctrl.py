# !/usr/bin/env python
# coding: utf-8
import math
import rospy
import threading
import Arm_Lib
from time import sleep
from snake_move import snake_move
from dofbot_info.srv import kinemarics, kinemaricsRequest, kinemaricsResponse


class snake_ctrl:
    def __init__(self):
        '''
        Initialize some parameters
        '''
        self.sbus = Arm_Lib.Arm_Device()
        self.arm_move = snake_move()
        self.color_name = None
        self.image = None
        self.cur_joint = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.Posture = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.move_time = 500

        self.grap_status = 'Waiting'
        # Angle of servo when clip close
        self.grap_joint = 145
        self.num = 1
        self.move_num = 1

        self.n = rospy.init_node('dofbot_snake', anonymous=True)

        self.client = rospy.ServiceProxy("dofbot_kinemarics", kinemarics)

    def read_joint(self):
        '''
        Read current angle of the servo in loop
        '''
        num = 0
        for i in range(1, 6):
            while 1:

                joint = self.sbus.Arm_serial_servo_read(i)

                # if num % 10 == 0: print("Please check!Poor contact!")

                if joint != None:
                    self.cur_joint[i - 1] = joint
                    break
                num += 1

        # print("current angle of the servo: {}".format(cur_joint))

    def get_Posture(self):
        '''
        Obtain angle of servo
        '''
        self.read_joint()

        self.client.wait_for_service()

        request = kinemaricsRequest()
        request.cur_joint1 = self.cur_joint[0]
        request.cur_joint2 = self.cur_joint[1]
        request.cur_joint3 = self.cur_joint[2]
        request.cur_joint4 = self.cur_joint[3]
        request.cur_joint5 = self.cur_joint[4]
        request.kin_name = "fk"
        try:
            response = self.client.call(request)
            if isinstance(response, kinemaricsResponse):

                self.Posture[0] = response.x
                self.Posture[1] = response.y
                self.Posture[2] = response.z
                self.Posture[3] = response.Roll
                self.Posture[4] = response.Pitch
                self.Posture[5] = response.Yaw
        except Exception:
            rospy.loginfo("get_Posture error")

    def joints_limit(self, joints):

        request = kinemaricsRequest()
        request.cur_joint1 = joints[0]
        request.cur_joint2 = joints[1]
        request.cur_joint3 = joints[2]
        request.cur_joint4 = joints[3]
        request.cur_joint5 = joints[4]
        request.kin_name = "fk"
        try:
            response = self.client.call(request)
            if isinstance(response, kinemaricsResponse):
                # print('response.y : ', response.y)
                # print('response.z : ', response.z)
                if 0.22 < response.z < 0.23 and response.y > 0 and -1.6 < response.Roll < -1.5:
                    move_joint1 = [90, joints[1], joints[2], joints[3], 0, 180]
                    move_joint2 = [90, joints[1], joints[2], joints[3], 0, 30]
                    if self.move_num == 1:
                        self.sbus.Arm_serial_servo_write6_array(move_joint1, self.move_time)
                        self.move_num = 2
                    if self.move_num == 2:
                        self.sbus.Arm_serial_servo_write6_array(move_joint2, self.move_time)
                        self.move_num = 1
        except Exception:
            rospy.loginfo("joints_limit error")

    def snake_run(self, point_y):
        '''
        Obtain angle of servo
        '''
        request = kinemaricsRequest()
        request.tar_x = self.Posture[0]
        request.tar_y = point_y
        request.tar_z = 0.225476
        request.kin_name = "ik"
        try:
            response = self.client.call(request)
            if isinstance(response, kinemaricsResponse):

                joints = [0.0, 0.0, 0.0, 0.0, 0.0]
                joints[0] = response.joint1
                joints[1] = response.joint2
                joints[2] = response.joint3
                joints[3] = response.joint4
                joints[4] = response.joint5
                for i in range(1, 6):
                    if joints[i - 1] < 0: return
                self.joints_limit(joints)
        except Exception:
            rospy.loginfo("snake_run error")

    def snake_main(self, name, msg):
        self.get_Posture()
        for key, area in msg.items():
            if key == name:

                distance = 27.05 * math.pow(area, -0.51) - 0.2

                target_dist = distance + self.Posture[1]
                if self.grap_status == 'Waiting':
                    threading.Thread(target=self.snake_run, args=(target_dist,)).start()
                    if self.Posture[1] <= 0.02:
                        # DOFBOT shakes head
                        self.sbus.Arm_serial_servo_write(5,180,300)
                        sleep(0.1)
                        self.sbus.Arm_serial_servo_write(5,0, 300)
                        sleep(0.1)
                        self.num = 1
                    elif self.Posture[1] >= 0.19:
                        # Open and close clip
                        self.sbus.Arm_serial_servo_write(6, 30, 100)
                        sleep(0.08)
                        self.sbus.Arm_serial_servo_write(6, 180, 100)
                        sleep(0.08)
                        self.num += 1
                    else:
                        # Open and close clip
                        self.sbus.Arm_serial_servo_write(6, 30, 100)
                        sleep(0.08)
                        self.sbus.Arm_serial_servo_write(6, 180, 100)
                        sleep(0.08)
                        self.num = 1
                    if self.num % 5 == 0: self.grap_status = 'Graping'
                elif self.grap_status == 'Graping':
                    self.grap_status = 'Runing'

                    self.arm_move.snake_run(name)

                    self.num = 1

                    self.grap_status = 'Waiting'
            else:
                if self.grap_status == 'Waiting':
                    # DOFBOT shakes head
                    self.sbus.Arm_serial_servo_write(5,180,300)
                    sleep(0.1)
                    self.sbus.Arm_serial_servo_write(5,0, 300)
                    sleep(0.1)
                    if self.Posture[1] > 0.01:
                        joint_1 = [90, 156, 16, 8, 0, 180]
                        self.sbus.Arm_serial_servo_write6_array(joint_1, self.move_time)
