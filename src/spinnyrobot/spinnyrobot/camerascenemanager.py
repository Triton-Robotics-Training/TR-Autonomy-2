import importlib.resources as resources
import random

import cv2
import numpy as np
import pybullet as p
import pybullet_data
from cv2 import cvtColor

sdf_file_path = str(resources.path('spinnyrobot.models', 'model.sdf'))


class CameraSceneManager:
    def __init__(self):
        self.client_id = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81, physicsClientId=self.client_id)
        self.plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client_id)
        self.box_id = p.loadURDF("cube.urdf", [-3, 0, .5], physicsClientId=self.client_id)
        p.changeVisualShape(self.box_id, -1, rgbaColor=[1, 0, 0, 1], physicsClientId=self.client_id)
        self.bot_id = p.loadSDF(sdf_file_path, physicsClientId=self.client_id)[0]
        p.resetBasePositionAndOrientation(self.bot_id, [0, 0, .5], [0, 0, 0, 1], physicsClientId=self.client_id)
        p.changeVisualShape(self.bot_id, 2, rgbaColor=[0, 0, 0, 0], physicsClientId=self.client_id)
        p.setRealTimeSimulation(True, physicsClientId=self.client_id)
        self.last_seen_slider_angle = 0.0
        self.slider_id = p.addUserDebugParameter("pos", -np.pi, np.pi, 0)

    def reset(self):
        p.resetBasePositionAndOrientation(self.box_id, [-3, (random.random() * -6) + 3, .5], [0, 0, 0, 1])
        p.setJointMotorControl2(self.bot_id, 1, controlMode=p.POSITION_CONTROL, targetPosition=0, force=500,
                                physicsClientId=self.client_id)
        p.resetJointState(self.bot_id, 1, 0)

    def spin_slider_angle(self):
        slider_angle = p.readUserDebugParameter(self.slider_id)
        if slider_angle != self.last_seen_slider_angle:
            p.setJointMotorControl2(self.bot_id, 1, controlMode=p.POSITION_CONTROL, targetPosition=slider_angle,
                                    force=500, physicsClientId=self.client_id)
        self.last_seen_slider_angle = slider_angle

    def set_angle(self, angle: float):
        p.setJointMotorControl2(self.bot_id, 1, controlMode=p.POSITION_CONTROL, targetPosition=angle, force=500,
                                physicsClientId=self.client_id)

    def get_angle(self):
        theta, *_ = p.getJointState(self.bot_id, 1, physicsClientId=self.client_id)
        return theta

    def render_image(self):
        cam_coords = np.array(p.getLinkState(self.bot_id, 2)[0])
        vm = p.computeViewMatrix(cameraEyePosition=cam_coords, cameraTargetPosition=np.multiply(cam_coords, [8, 8, 1]),
                                 cameraUpVector=[0, 0, 1])
        pm = p.computeProjectionMatrixFOV(90, 1, .1, 100)
        _, _, image, *_ = p.getCameraImage(width=640, height=640, viewMatrix=vm, projectionMatrix=pm,
                                           renderer=p.ER_TINY_RENDERER)
        image = image[:, :, :3]
        image = cvtColor(image, cv2.COLOR_RGB2BGR)
        return image
