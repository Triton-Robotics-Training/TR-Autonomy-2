import time

import cv2 as cv
import numpy as np

from spinnyrobot.camerascenemanager import CameraSceneManager


def image_centroid(image):
    M = cv.moments(image)

    # calculate x,y coordinate of center
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        cX, cY = 0, 0
    return np.array([cX, cY])


def filter_hsv(hsvimage, color, dist):
    hue = hsvimage[:, :, 0]
    sat = hsvimage[:, :, 1]
    val = hsvimage[:, :, 2]
    h, s, v = color
    centering = 90 - h
    hue += centering
    hue = np.mod(hue, 180)
    lower = np.array([90 - dist, 100, 100])
    upper = np.array([90 + dist, 255, 255])
    adjimg = np.dstack((hue, sat, val))
    mask = cv.inRange(adjimg, lower, upper)
    return mask


class TaskManager:
    def __init__(self):
        self.csm = CameraSceneManager()
        self.csm.reset()
        self.timecounter = 1
        self.last_spun_time = time.time()
        self.points = 0

    def spin_once(self):
        self.csm.spin_slider_angle()
        image = self.csm.render_image()
        hsvimage = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        himage = filter_hsv(hsvimage, [0, 255, 153], 20)
        x, y = image_centroid(himage)

        center = 320
        dist = 60

        delta = time.time() - self.last_spun_time
        self.last_spun_time = time.time()

        if center - dist < x < center + dist:
            self.timecounter += delta
        if self.timecounter >= 2:
            self.csm.reset()
            self.points += 1
            print(f"Gained one point! You have {self.points} point(s).")
            self.timecounter = 0

    def render_image(self):
        return self.csm.render_image()

    def get_joint_angle(self):
        return self.csm.get_angle()

    def set_joint_angle(self, angle: float):
        return self.csm.set_angle(angle)
