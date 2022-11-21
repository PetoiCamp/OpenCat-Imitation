#!/usr/bin/env python3
from dataclasses import dataclass
import numpy as np
from serialMaster import ardSerial
import threading
import time

sendCmd = True
mirror = False
E_RGB_ALL = 0
E_RGB_RIGHT = 1
E_RGB_LEFT = 2
E_EFFECT_BREATHING = 0
E_EFFECT_ROTATE = 1
E_EFFECT_FLASH = 2
E_EFFECT_NONE = 3


angleGoal = [0,0,0,0,0,0,]
#self.angles['neck_h'],self.angles['neck_v'], self.angles['shoulder_l'], self.angles['shoulder_r'],self.angles['elbow_l'],self.angles['elbow_r']]

@dataclass
class Model:
    """
    storing position of keypoints
    """
    nose: np.array
    left_shoulder: np.array
    right_shoulder: np.array
    left_hip: np.array
    right_hip: np.array
    left_elbow: np.array
    right_elbow: np.array
    left_wrist: np.array
    right_wrist: np.array
    thr: float


class Cat:

    def __init__(self):
        """
        init connection
        """
        # human pose records
        # TODO: set init position for joints
        self.angles: dict = {
            'neck_h': 0,  # horizontal neck angle
            'neck_v': 0,  # vertical neck angle
            'shoulder_l': 0,  # left shoulder joint
            'shoulder_r': 0,  # right shoulder joint
            'elbow_l': 0,  # left elbow joint
            'elbow_r': 0,  # right elbow joint
        }
        #        model = input('What is your model? (Nybble/Bittle)\n')
        # prepare pose for Nybble
#        if model =='Nybble':
        prepare = [
            -4,   0,   0,   1,
            0,   0,   0,
            0,   0,   0,   0,   0,   0,   0,   0,  30,  30, -30, -30,  30,  30, -30, -30,   8,   0,   0,   0,
            10, -20,  75,   0,  -5,  -5,  20,  20,  30,  30, -90, -90,  60,  60,  -35,  -35,   8,   0,   0,   0,
            10, -20,  80,   0,  -5,  -5,  20,  20,  42,  42, -90, -90,  78,  78,  -35,  -35,   8,   0,   0,   0,
            0, -54,  80,   0,  -5,  -5,  20,  20, -34, -47, -60, -60,  47,  78,  -35,  -35,   8,   0,   0,   0,
            ]
        if sendCmd:
            self.goodPorts = {}
            ardSerial.connectPort(self.goodPorts)
            self.t = threading.Thread(target=ardSerial.keepCheckingPort,
                                      args=(self.goodPorts, ))
            self.t.start()
        # prepare pose for Bittle
        # ardSerial.send(self.goodPorts, ['kcalib', 1])
        # ardSerial.send(self.goodPorts,
        #                ['I', [0, 0, 10, -52, 11, -52, 14, 83, 15, 83], 1])
        # ardSerial.send(self.goodPorts,
        #                ['I', [0, 0, 10, -15, 11, -15, 14, 83, 15, 83], 1])
        
            ardSerial.send(self.goodPorts,['g', 0.5])

            ardSerial.send(self.goodPorts,['K', prepare, 1])

        # start control loop
        self.control_t = threading.Thread(target=self.control_loop)
        self.mtx = threading.Lock()
        self.control_t.start()
        self.updated = False

    def control_loop(self):
        if sendCmd:
            ardSerial.send(self.goodPorts, ['C',[0,  0,  127, E_RGB_ALL,   E_EFFECT_FLASH],  0])
        anglelist = []
        colorState = 0;
        while True:
            timer = []
            
            with self.mtx:
                timer.append(time.perf_counter())
                if anglelist == list(self.angles.values()):
                    print("same")
                    continue
    #                print('**',end = '')
    #                print(anglelist,end = '\n**')
    #                print(list(self.angles.values()))
                if mirror:
                    cmd: list = [
                    'I',
                        [
                        0, -self.angles['neck_h'], 1, self.angles['neck_v'], 9, self.angles['shoulder_l'],
                        8, self.angles['shoulder_r'], 13,
                        self.angles['elbow_l'], 12, self.angles['elbow_r']
                        ], 0
                    ]
                else:
                    cmd: list = [
                    'I',
                        [
                        0, self.angles['neck_h'], 1, self.angles['neck_v'], 8, self.angles['shoulder_l'],
                        9, self.angles['shoulder_r'], 12,
                        self.angles['elbow_l'], 13, self.angles['elbow_r']
                        ], 0
                    ]
                #                print(list(self.angles.values()))
                
                if sendCmd:
                    timer.append(time.perf_counter())
                    if self.angles['neck_v']>-15:
                        if colorState !=1:
                            ardSerial.send(self.goodPorts, ['C',[127,  0,  0, E_RGB_ALL,   E_EFFECT_FLASH],  0.0])
                            colorState = 1
                    elif self.angles['neck_v']<-60:
                        if colorState !=2:
                            ardSerial.send(self.goodPorts, ['C',[0, 127,  127, E_RGB_ALL,   E_EFFECT_FLASH],  0.0])
                            colorState = 2
                    else:
                        if colorState !=3:
                            ardSerial.send(self.goodPorts, ['C',[0, 127,  0,  E_RGB_ALL,   E_EFFECT_FLASH],  0.0])
                            colorState = 3
                    timer.append(time.perf_counter())
                    
                    ardSerial.send(self.goodPorts, cmd)
                    timer.append(time.perf_counter())
                    
#                    print(timer)
                    
                anglelist = list(self.angles.values())

    def control_cat(self, model: Model):
        """
        control opencat based on human pose
        Input:
          model: Model sent as message
        """
        # update neck angle
        with self.mtx:
#            timer = []
#            timer.append(time.perf_counter())
            if self._check_thresh(model.thr, model.nose[3],
                                  model.left_shoulder[3],
                                  model.right_shoulder[3]):
                neck_angles = self._get_neck_angle(
                    model.left_shoulder[:3], model.right_shoulder[:3],
                    model.nose[:3])
                self.angles['neck_h'] = neck_angles[0]
                self.angles['neck_v'] = neck_angles[1]-75
#            timer.append(time.perf_counter())

            # update shoulder l
            if self._check_thresh(model.thr, model.left_shoulder[3],
                                  model.left_hip[3], model.left_elbow[3]):
                self.angles['shoulder_l'] = 90 - self._vec_angle(
                    (model.left_hip - model.left_shoulder)[:3],
                    (model.left_elbow - model.left_shoulder)[:3]) - 30
#            timer.append(time.perf_counter())

            # update shoulder r
            if self._check_thresh(model.thr, model.right_shoulder[3],
                                  model.right_hip[3], model.right_elbow[3]):
                self.angles['shoulder_r'] = 90 - self._vec_angle(
                    (model.right_hip - model.right_shoulder)[:3],
                    (model.right_elbow - model.right_shoulder)[:3]) - 30
#            timer.append(time.perf_counter())

            # update elbow l
            if self._check_thresh(model.thr, model.left_shoulder[3],
                                  model.left_wrist[3], model.left_elbow[3]):
                self.angles['elbow_l'] = self._vec_angle(
                    (model.left_shoulder - model.left_elbow)[:3],
                    (model.left_wrist - model.left_elbow)[:3]) - 75
#            timer.append(time.perf_counter())

            # update elbow r
            if self._check_thresh(model.thr, model.right_shoulder[3],
                                  model.right_wrist[3], model.right_elbow[3]):
                self.angles['elbow_r'] = self._vec_angle(
                    (model.right_shoulder - model.right_elbow)[:3],
                    (model.right_wrist - model.right_elbow)[:3]) - 75
#            timer.append(time.perf_counter())
#            print(timer)

    def _check_thresh(self, thr: float, *values) -> bool:
        """
        check for each value in args whether val >= threshold
        Inputs:
          thr: threshold
          values: list of values to compare
        """
        for val in values:
            if val < thr:
                return False

        return True

    def _get_neck_angle(self, left_shoulder, right_shoulder, nose) -> float:
        """
        Calculate neck angle
        """
        # get mid point of shoulder
        shoulder_mid = (left_shoulder + right_shoulder) / 2
        # mid -> nose
        mid_nose = nose - shoulder_mid
        # left shoulder -> mid
        left_mid = shoulder_mid - left_shoulder
        # get normal vector to plane
        plane_normal = np.cross(left_mid, mid_nose)
        front_dir = np.cross(plane_normal, left_mid)
        pan = self._vec_angle(front_dir, mid_nose)
        # determine direction
        axis = 1
        if np.inner(np.cross(mid_nose, front_dir), plane_normal) > 0:
            axis = -1

        tilt_angle = self._vec_angle(mid_nose,left_mid)
        # print(tilt_angle,end = '\t')
        tilt_angle = np.deg2rad(tilt_angle)
#        tilt = np.linalg.norm(mid_nose)*np.sin(tilt_angle)
        shoulderLength = np.linalg.norm(left_mid)
        tilt = (np.linalg.norm(mid_nose)*np.sin(tilt_angle)-shoulderLength*2/3)/(shoulderLength/3)
        tilt = np.rad2deg(np.arctan(tilt))

        return [axis * pan,tilt]

    def _vec_angle(self, a: np.array, b: np.array) -> float:
        """
        Calculate angle between vector a and b
        """
        import numpy.linalg as LA
        inner = np.inner(a, b)
        norms = LA.norm(a) * LA.norm(b)

        cos = inner / norms
        rad = np.arccos(np.clip(cos, -1.0, 1.0))
        deg = np.rad2deg(rad)
        return deg
