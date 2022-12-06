#!/usr/bin/env python3
from dataclasses import dataclass
import numpy as np
from serialMaster import ardSerial
import threading
import time

sendCmd = True
mirror = False # flip the left-right mapping on the robot
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
        self.prev_angles:angleList = np.array([0,0,0,0,0,0,0])
        self.angles: dict = {
            'neck_h': 0,  # horizontal neck angle
            'neck_v': 0,  # vertical neck angle
            'shoulder_l': 0,  # left shoulder joint
            'shoulder_r': 0,  # right shoulder joint
            'elbow_l': 0,  # left elbow joint
            'elbow_r': 0,  # right elbow joint
            'spine':0
        }
        self.updated = False
        self.model = input('What is your model? (Nybble/Bittle)\n')
        # prepare pose for Nybble
        if self.model =='Nybble':
            prepare = [
                -5,   0,   0,   1,
                 0,   0,   0,
               -30, -80, -45,   0,  -3,  -3,   3,   3,  60,  60, -60, -60, -45, -45,  45,  45,   8,   0,   0,   0,
                 0,   0,   0,   0,   0,   0,   0,   0,  30,  30, -30, -30,  30,  30, -30, -30,   8,   0,   0,   0,
                10, -20,  75,   0,  -5,  -5,  20,  20,  30,  30, -90, -90,  60,  60,  -35,  -35,   8,   0,   0,   0,
                10, -20,  80,   0,  -5,  -5,  20,  20,  42,  42, -90, -90,  78,  78,  -35,  -35,   8,   0,   0,   0,
                 0, -54,  100,   0,  -5,  -5,  20,  20, -47, -47, -65, -65,  47,  78,  -30,  -30,   8,   0,   0,   0,
                ]
        # prepare pose for Bittle
        elif self.model =='Bittle':
            prepare = [
              -5,   0,   0,   1,
               0,   0,   0,
             -30, -80, -45,   0,  -3,  -3,   3,   3,  70,  70,  70,  70, -55, -55, -55, -55,   8,   0,   0,   0,
               0,   0,   0,   0,   0,   0,   0,   0,  30,  30,  30,  30,  30,  30,  30,  30,   8,   0,   0,   0,
               0,   0,   0,   0,   0,   0,   0,   0,   4,   4,   9,   9,  85,  85,  41,  41,   8,   0,   0,   0,
               0,   0,   0,   0,   0,   0,   0,   0,  31,  31, -26, -26,  86,  86,  78,  78,   4,   0,   0,   0,
               0,   0,   0,   0,   0,   0,   0,   0, -34, -34, -25, -25,  30,  30,  85,  85,   8,   0,   0,   0,
            ]
        if sendCmd:
            self.goodPorts = {}
            ardSerial.connectPort(self.goodPorts)
            self.t = threading.Thread(target=ardSerial.keepCheckingPort,
                                      args=(self.goodPorts, ))
            self.t.start()

            time.sleep(2)
            ardSerial.send(self.goodPorts,['g', 0.5])

            ardSerial.send(self.goodPorts,['K', prepare, 1])

        # start control loop
        self.control_t = threading.Thread(target=self.control_loop)
        self.mtx = threading.Lock()
        self.control_t.start()


    def control_loop(self):
        if sendCmd:
            ardSerial.send(self.goodPorts, ['C',[0,  0,  127, E_RGB_ALL,   E_EFFECT_FLASH],  0])
        
        colorState = 0;
        while True:
            timer = []
            timer.append(time.perf_counter())
            change = np.linalg.norm(self.prev_angles-list(self.angles.values()))
            if self.updated and change>3:
#                print(change)
#                print(self.prev_angles,end = ',\t')
#                print(list(self.angles.values()))
#                print(self.prev_angles-np.array(list(self.angles.values())))
    #                print('**',end = '')
    #                print(anglelist,end = '\n**')
    #                print(list(self.angles.values()))
                hp,ht,sFL,sFR,sBR, sBL, kFL, kFR, kBR, kBL = 0,1,8,9,10,11,12,13,14,15

                if self.model == 'Nybble':
                    cmd: list = [
                    'I',
                        [
                        hp, self.angles['neck_h'], ht, self.angles['neck_v'],
                        sFL, self.angles['shoulder_l'],sFR, self.angles['shoulder_r'],
                        kFL,self.angles['elbow_l'], kFR, self.angles['elbow_r'],
                        sBL,-65 - self.angles['spine'],sBR, -65 + self.angles['spine'],
                        kBL, -30 + self.angles['spine']/2,kBR,-30 - self.angles['spine']/2,
                        ], 0
                    ]
                elif self.model == 'Bittle':
                    cmd: list = [
                    'I',
                        [
                        hp, self.angles['neck_h'], ht, self.angles['neck_v'],
                        sFL, self.angles['shoulder_l'], sFR, self.angles['shoulder_r'],
                        kFL, self.angles['elbow_l'], kFR, self.angles['elbow_r'],
                        sBL,-34 - self.angles['spine'],sBR, -34+ self.angles['spine'],
                        kBL, 85 + self.angles['spine']/3,kBR, 85 - self.angles['spine']/3,
                        ], 0
                    ]
                #                print(list(self.angles.values()))
                if mirror:
                    cmd[1][1]=-cmd[1][1]
                    for i in range(1,len(cmd[1])//4):
                        temp = cmd[1][i*4]
                        cmd[1][i*4]=cmd[1][i*4+2]
                        cmd[1][i*4+2]=temp
                
                if sendCmd:
                    timer.append(time.perf_counter())
                    if self.angles['neck_v']>-15:
                        if colorState !=1:
                            ardSerial.send(self.goodPorts, ['C',[64, 0, 127, E_RGB_ALL,   E_EFFECT_FLASH],  0.0])
                            colorState = 1
                    elif self.angles['neck_v']<-55:
                        if colorState !=2:
                            ardSerial.send(self.goodPorts, ['C',[0, 127, 127, E_RGB_ALL,   E_EFFECT_FLASH],  0.0])
                            colorState = 2
                    else:
                        if colorState !=3:
                            ardSerial.send(self.goodPorts, ['C',[64, 127, 0,   E_RGB_ALL,   E_EFFECT_FLASH],  0.0])
                            colorState = 3
                    timer.append(time.perf_counter())
                    
                    ardSerial.send(self.goodPorts, cmd)
                    timer.append(time.perf_counter())
                self.prev_angles = np.array(list(self.angles.values()))
                self.updated = False
                    
#                    print(timer)
                
    
    
    
    def control_cat(self, model: Model):
        """
        control opencat based on human pose
        Input:
          model: Model sent as message
        """
        # update neck angle
#            timer = []
#            timer.append(time.perf_counter())
        smallAngleThreshold = 2 # filter out shakes in the algorithm
        if self._check_thresh(model.thr, model.nose[3],
                              model.left_shoulder[3],
                              model.right_shoulder[3]):
            neck_angles = self._get_neck_angle(
                model.left_shoulder[:3], model.right_shoulder[:3],
                model.nose[:3])
            if abs(neck_angles[0]-self.angles['neck_h'])>smallAngleThreshold:
                self.angles['neck_h'] = neck_angles[0]
            if abs(neck_angles[0]-self.angles['neck_v'])>smallAngleThreshold:
                self.angles['neck_v'] = neck_angles[1]
#            timer.append(time.perf_counter())

        # update shoulder l
        if self._check_thresh(model.thr, model.left_shoulder[3],
                              model.left_hip[3], model.left_elbow[3]):
            shoulderL = 60 - self._vec_angle(
                (model.left_hip - model.left_shoulder)[:3],
                (model.left_elbow - model.left_shoulder)[:3])
            if abs(shoulderL -self.angles['shoulder_l'])>smallAngleThreshold:
                self.angles['shoulder_l'] = shoulderL
#            timer.append(time.perf_counter())

        # update shoulder r
        if self._check_thresh(model.thr, model.right_shoulder[3],
                              model.right_hip[3], model.right_elbow[3]):
            shoulderR = 60 - self._vec_angle(
                (model.right_hip - model.right_shoulder)[:3],
                (model.right_elbow - model.right_shoulder)[:3])
            if abs(shoulderR-self.angles['shoulder_r'])>smallAngleThreshold:
                self.angles['shoulder_r'] = shoulderR
#            timer.append(time.perf_counter())

        # update elbow l
        if self._check_thresh(model.thr, model.left_shoulder[3],
                              model.left_wrist[3], model.left_elbow[3]):
            elbowL = self._vec_angle(
                (model.left_shoulder - model.left_elbow)[:3],
                (model.left_wrist - model.left_elbow)[:3]) - 75
            if abs(elbowL - self.angles['elbow_l'])>smallAngleThreshold:
                self.angles['elbow_l'] = elbowL
#            timer.append(time.perf_counter())

        # update elbow r
        if self._check_thresh(model.thr, model.right_shoulder[3],
                              model.right_wrist[3], model.right_elbow[3]):
            elbowR = self._vec_angle(
                (model.right_shoulder - model.right_elbow)[:3],
                (model.right_wrist - model.right_elbow)[:3]) - 75
            if abs(elbowR-self.angles['elbow_r'])>smallAngleThreshold:
                self.angles['elbow_r'] = elbowR
                
        if self._check_thresh(model.thr, model.right_shoulder[3],
                            model.left_shoulder[3], model.right_hip[3],model.left_hip[3]):
            
            spine = self._vec_angle(
                (model.right_shoulder - model.left_shoulder)[:3],
                (model.right_hip - model.left_hip)[:3])/4
            direction = np.sign(np.linalg.norm((model.right_shoulder-model.right_hip)[:3]) - np.linalg.norm((model.left_shoulder-model.left_hip)[:3]))
            spine *= direction
            
            if abs(spine-self.angles['spine'])>smallAngleThreshold:
                self.angles['spine'] = spine
                print(self.angles['spine'])
#            timer.append(time.perf_counter())
#            print(timer)
        self.updated = True

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

        return [axis * pan,tilt-75]

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
