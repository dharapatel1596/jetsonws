#!/usr/bin/env python
# coding: utf-8
import rospy
import Arm_Lib
import cv2 as cv
from math import pi
from time import sleep
from stacking_grap import stacking_grap
from dofbot_info.srv import kinemarics, kinemaricsRequest, kinemaricsResponse


class stacking_GetTarget:
    def __init__(self):
        self.image = None
        self.color_name = None
        self.color_status = True

        self.xy = [90, 135]

        self.arm = Arm_Lib.Arm_Device()

        self.grap = stacking_grap()

        self.n = rospy.init_node('dofbot_stacking', anonymous=True)

        self.client = rospy.ServiceProxy("dofbot_kinemarics", kinemarics)

    def target_run(self, msg, xy=None):

        if xy != None: self.xy = xy
        num = 1
        move_status=0
        for i in msg.values():
            if i !=None: move_status=1
        if move_status==1:
            self.arm.Arm_Buzzer_On(1)
            sleep(0.5)
        for name, pos in msg.items():
            try:
                joints = self.server_joint(pos)
                self.grap.arm_run(str(num), joints)
                num += 1
            except Exception:
                print("sqaure_pos empty")

        self.arm.Arm_serial_servo_write(1, 90, 1000)
        sleep(1)

        joints_0 = [self.xy[0], self.xy[1], 0, 0, 90, 30]

        self.arm.Arm_serial_servo_write6_array(joints_0, 1000)
        sleep(1)

    def select_color(self, image, color_hsv, color_list):

        self.image = cv.resize(image, (640, 480))
        msg = {}
        if len(color_list) == 0: return self.image, msg
        if '4' in color_list:
            self.color_name = color_list['4']
            pos = self.get_Sqaure(color_hsv[self.color_name])
            if pos != None: msg[self.color_name] = pos
        if '3' in color_list:
            self.color_name = color_list['3']
            pos = self.get_Sqaure(color_hsv[self.color_name])
            if pos != None: msg[self.color_name] = pos
        if '2' in color_list:
            self.color_name = color_list['2']
            pos = self.get_Sqaure(color_hsv[self.color_name])
            if pos != None: msg[self.color_name] = pos
        if '1' in color_list:
            self.color_name = color_list['1']
            pos = self.get_Sqaure(color_hsv[self.color_name])
            if pos != None: msg[self.color_name] = pos
        return self.image, msg

    def get_Sqaure(self, color_hsv):

        (lowerb, upperb) = color_hsv

        mask = self.image.copy()

        hsv = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)

        img = cv.inRange(hsv, lowerb, upperb)

        mask[img == 0] = [0, 0, 0]

        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))

        dst_img = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

        dst_img = cv.cvtColor(dst_img, cv.COLOR_RGB2GRAY)

        ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)

        contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)  
        for i, cnt in enumerate(contours):

            x, y, w, h = cv.boundingRect(cnt)

            area = cv.contourArea(cnt)

            if area > 1000:

                point_x = float(x + w / 2)
                point_y = float(y + h / 2)

                cv.rectangle(self.image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                cv.circle(self.image, (int(point_x), int(point_y)), 5, (0, 0, 255), -1)

                cv.putText(self.image, self.color_name, (int(x - 15), int(y - 15)),
                           cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

                (a, b) = (round(((point_x - 320) / 4000), 5), round(((480 - point_y) / 3000) * 0.8+0.19, 5))
                return (a, b)

    def server_joint(self, posxy):


        self.client.wait_for_service()

        request = kinemaricsRequest()
        request.tar_x = posxy[0]
        request.tar_y = posxy[1]
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

                if joints[2] < 0:
                    joints[1] += joints[2] * 3 / 5
                    joints[3] += joints[2] * 3 / 5
                    joints[2] = 0
                # print joints
                return joints
        except Exception:
            rospy.loginfo("arg error")
