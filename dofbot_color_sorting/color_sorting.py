#!/usr/bin/env python
# coding: utf-8
import random
import Arm_Lib
import threading
import cv2 as cv
from time import sleep


class color_sorting:
    def __init__(self):
        '''
        Set initialization parameters
        '''
        self.image = None
        self.num = 0

        self.status = 'waiting'

        self.arm = Arm_Lib.Arm_Device()

        self.grap_joint = 135

        self.joints = [90, 53, 33, 36, 90, 30]

    def get_Sqaure(self, color_name, hsv_lu):
        '''
        Color identification
        '''
        (lowerb, upperb) = hsv_lu

        point_Xmin=50   #200
        point_Xmax=600  #440
        point_Ymin=80   #200
        point_Ymax=480  #480

#         cv.rectangle(self.image, (point_Xmin, point_Ymin), (point_Xmax, point_Ymax),(105,105,105), 2)

        img = self.image.copy()
        mask = img[point_Ymin:point_Ymax, point_Xmin:point_Xmax]
#         mask = self.image.copy()

        HSV_img = cv.cvtColor(mask, cv.COLOR_BGR2HSV)
 
        img = cv.inRange(HSV_img, lowerb, upperb)

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

                x_w_ = float(x + w / 2)
                y_h_ = float(y + h / 2)

                cv.rectangle(self.image, (x + point_Xmin, y + point_Ymin), (x + w + point_Xmin, y + h + point_Ymin), (0, 255, 0), 2)

                cv.circle(self.image, (int(x_w_ + point_Xmin), int(y_h_ + point_Ymin)), 5, (0, 0, 255), -1)

                cv.putText(self.image, color_name, (int(x + point_Xmin-15), int(y + point_Ymin-15)), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                
#                 cv.rectangle(self.image, (x , y ), (x + w , y + h ), (0, 255, 0), 2)
#                 cv.circle(self.image, (int(x_w_ ), int(y_h_ )), 5, (0, 0, 255), -1)
#                 cv.putText(self.image, color_name, (int(x -15), int(y -15)), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

                return (x_w_, y_h_)

    def Sorting_grap(self, img, color_hsv):

        self.image = cv.resize(img, (640, 480))

        msg = {}
        if self.status == 'waiting':

            for key, value in color_hsv.items():
                point = self.get_Sqaure(key, value)
                if point != None: msg["name"] = key
            if len(msg) == 1:
                self.num += 1

                if self.num % 10 == 0 and self.status == 'waiting':
                    self.status = "Runing"
                    self.arm.Arm_Buzzer_On(1)
                    sleep(0.5)

                    threading.Thread(target=self.sorting_run, args=(msg['name'],)).start()
                    self.num = 0
        return self.image

    def sorting_move(self, joints_target):

        joints_up = [90, 80, 35, 40, 90, self.grap_joint]

        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        sleep(1)

        self.arm.Arm_serial_servo_write(6, 0, 500)
        sleep(0.5)

        self.arm.Arm_serial_servo_write6_array(self.joints, 1000)
        sleep(1)

        self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        sleep(0.5)

        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        sleep(1)
 
        self.arm.Arm_serial_servo_write(1, joints_target[0], 500)
        sleep(0.5)

        self.arm.Arm_serial_servo_write6_array(joints_target, 1000)
        sleep(1.5)

        self.arm.Arm_serial_servo_write(6, 0, 500)
        sleep(0.5)

        joints_up[0] = joints_target[0]
        self.arm.Arm_serial_servo_write6_array(joints_up, 500)
        sleep(0.5)

        self.arm.Arm_serial_servo_write(1, 90, 500)
        sleep(0.5)

        joints_0 = [90, 130, 0, 0, 90, 0]

        self.arm.Arm_serial_servo_write6_array(joints_0, 1000)
        sleep(1)

    def sorting_run(self, name):

        if name == "red" :
            # print("red")

            # joints_target = [115, 20, 80, 40, 90, self.grap_joint]
            joints_target = [117, 19, 66, 56, 90, self.grap_joint]

            self.sorting_move(joints_target)

            self.status = 'waiting'
        if name == "blue":
            # print("blue")
            # joints_target = [45, 80, 0, 40, 90, self.grap_joint]
            joints_target = [44, 66, 20, 28, 90, self.grap_joint]
            self.sorting_move(joints_target)

            self.status = 'waiting'
        if name == "green" :
            # print("green")
            # joints_target = [137, 80, 0, 40, 90, self.grap_joint]
            joints_target = [136, 66, 20, 29, 90, self.grap_joint]
            self.sorting_move(joints_target)

            self.status = 'waiting'
        if name == "yellow" :
            # print("yellow")
            # joints_target = [65, 20, 80, 40, 90, self.grap_joint]
            joints_target = [65, 22, 64, 56, 90, self.grap_joint]
            self.sorting_move(joints_target)

            self.status = 'waiting'

