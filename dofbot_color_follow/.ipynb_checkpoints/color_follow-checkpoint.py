# !/usr/bin/env python
# coding: utf-8
import cv2 as cv
import PID
import Arm_Lib


class color_follow:
    def __init__(self):
        '''
        Initialize some parameters
        '''
        self.img = None
        self.target_servox=90
        self.target_servoy=45
        self.Arm = Arm_Lib.Arm_Device()
        self.xservo_pid = PID.PositionalPID(1.9, 0.3, 0.35)
        self.yservo_pid = PID.PositionalPID(1.9, 0.3, 0.35)

    def follow_function(self, img, HSV_config):
        (color_lower, color_upper) = HSV_config
        self.img = cv.resize(img, (640, 480), )
        self.img = cv.GaussianBlur(self.img, (5, 5), 0)
        hsv = cv.cvtColor(self.img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, color_lower, color_upper)
        mask = cv.erode(mask, None, iterations=2)
        mask = cv.dilate(mask, None, iterations=2)
        mask = cv.GaussianBlur(mask, (5, 5), 0)
        cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts) > 0:
            cnt = max(cnts, key=cv.contourArea)
            (color_x, color_y), color_radius = cv.minEnclosingCircle(cnt)
            if color_radius > 10:

                cv.circle(self.img, (int(color_x), int(color_y)), int(color_radius), (255, 0, 255), 2)
#                 cv.putText(self.img, follow_name, (300, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (105, 105, 105), 2)
                self.xservo_pid.SystemOutput = color_x
                self.xservo_pid.SetStepSignal(320)
                self.xservo_pid.SetInertiaTime(0.01, 0.1)
                if not (self.target_servox>=180 and color_x<=320 or self.target_servox<=0 and color_x>=320):
                    self.xservo_pid.SystemOutput = color_x
                    self.xservo_pid.SetStepSignal(320)
                    self.xservo_pid.SetInertiaTime(0.01, 0.1)
                    target_valuex = int(1500 + self.xservo_pid.SystemOutput)
                    self.target_servox = int((target_valuex - 500) / 10)

                    if self.target_servox > 180:self.target_servox = 180
                    if self.target_servox < 0: self.target_servox = 0
                if not (self.target_servoy>=180 and color_y<=240 or self.target_servoy<=0 and color_y>=240):

                    self.yservo_pid.SystemOutput = color_y
                    self.yservo_pid.SetStepSignal(240)
                    self.yservo_pid.SetInertiaTime(0.01, 0.1)
                    target_valuey = int(1500 + self.yservo_pid.SystemOutput)
                    self.target_servoy = int((target_valuey - 500) / 10) - 45

                    if self.target_servoy > 360: self.target_servoy = 360
                    if self.target_servoy < 0: self.target_servoy = 0
                joints_0 = [self.target_servox, 135, self.target_servoy / 2, self.target_servoy / 2, 90, 30]
                self.Arm.Arm_serial_servo_write6_array(joints_0, 1000)
        return self.img

    def get_hsv(self, img):
        '''
        Gets the range of the HSV for a region
        :param img: color image
        :return: Range of image and HSV
        '''
        H = [];S = [];V = []
        img = cv.resize(img, (640, 480), )

        HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        point_init_x=290
        point_init_y=300
        point_end_x=350
        point_end_y=360
        cv.rectangle(img, (point_init_x, point_init_y), (point_end_x, point_end_y), (0, 255, 0), 2)

        for i in range(point_init_x, point_end_x):
            for j in range(point_init_y, point_end_y):
                H.append(HSV[j, i][0])
                S.append(HSV[j, i][1])
                V.append(HSV[j, i][2])

        H_min = min(H);H_max = max(H)
        S_min = min(S);S_max = max(S)
        V_min = min(V);V_max = max(V)

        if H_min - 2 < 0:H_min = 0
        else:H_min -= 2
        if S_min - 15 < 0:S_min = 0
        else:S_min -= 15
        if V_min - 15 < 0:V_min = 0
        else:V_min -= 15
        if H_max + 2 > 255:H_max = 255
        else:H_max += 2
        S_max = 255;V_max = 255
        lowerb = 'lowerb : (' + str(H_min) + ' ,' + str(S_min) + ' ,' + str(V_min) + ')'
        upperb = 'upperb : (' + str(H_max) + ' ,' + str(S_max) + ' ,' + str(V_max) + ')'
        txt1 = 'Learning ...'
        txt2 = 'OK !!!'
        if S_min < 5 or V_min < 5:cv.putText(img, txt1, (230, 270), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        else:cv.putText(img, txt2, (270, 270), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv.putText(img, lowerb, (150, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv.putText(img, upperb, (150, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        hsv_range = ((int(H_min), int(S_min), int(V_min)), (int(H_max), int(S_max), int(V_max)))
        return img, hsv_range

    def learning_follow(self, img, HSV_config):
        img = self.follow_function(img, HSV_config)
        return img
