# !/usr/bin/env python
# coding: utf-8
import cv2 as cv
import numpy as np


class snake_target:
    def __init__(self):
        '''
        Initialize some parameters
        '''
        self.image = None
        self.cur_joint = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.Posture = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def Image_Processing(self, img):

        gray_img = cv.cvtColor(img, cv.COLOR_RGB2GRAY)

        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))

        dst_img = cv.morphologyEx(gray_img, cv.MORPH_CLOSE, kernel)

        ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)

        # cv.imshow("th", binary)
        # _, contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) #python2
        contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)  # python3
        return contours

    def get_area(self, hsv_name, hsv_range):
        (lowerb, upperb) = hsv_range

        color_mask = self.image.copy()

        hsv_img = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)

        color = cv.inRange(hsv_img, lowerb, upperb)

        color_mask[color == 0] = [0, 0, 0]
        # cv.imshow("mask", color_mask)

        contours = self.Image_Processing(color_mask)

        for i, cnt in enumerate(contours):

            mm = cv.moments(cnt)
            if mm['m00'] == 0:
                continue
            cx = mm['m10'] / mm['m00']
            cy = mm['m01'] / mm['m00']

            (x, y) = (np.int(cx), np.int(cy))

            area = cv.contourArea(cnt)

            if area > 800:

                cv.circle(self.image, (x, y), 5, (0, 0, 255), -1)

                rect = cv.minAreaRect(cnt)

                box = cv.boxPoints(rect)

                box = np.int0(box)

                cv.drawContours(self.image, [box], 0, (255, 0, 0), 2)

                cv.putText(self.image, hsv_name, (int(box[1][0] - 15), int(box[1][1]) - 15),
                           cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                return area

    def target_run(self, img, color_hsv):

        self.image = cv.resize(img, (640, 480), )

        msg = {}
        for key, value in color_hsv.items():
            area = self.get_area(key, value)
            if area != None: msg[key] = area
        return self.image, msg
