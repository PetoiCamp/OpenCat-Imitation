#!/usr/bin/env python3
from dataclasses import dataclass
import numpy as np
from serialMaster import ardSerial
import threading


@dataclass
class Model:
    """
    storing position of keypoints
    """
    nose: np.array
    left_shoulder: np.array
    right_shoulder: np.array
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
        self.goodPorts = {}
        ardSerial.connectPort(self.goodPorts)
        self.t = threading.Thread(target=ardSerial.keepCheckingPort,
                                  args=(self.goodPorts, ))
        self.t.start()
        # prepare pose
        ardSerial.send(self.goodPorts, ['kcalib', 1])
        ardSerial.send(self.goodPorts,
                       ['I', [0, 0, 10, -52, 11, -52, 14, 83, 15, 83], 1])
        ardSerial.send(self.goodPorts,
                       ['I', [0, 0, 10, -15, 11, -15, 14, 83, 15, 83], 1])

    def control_cat(self, model: Model):
        """
        control opencat based on human pose
        Input:
          model: Model sent as message
        """
        # update neck angle
        if self._check_thresh(model.thr, model.nose[3], model.left_shoulder[3],
                              model.right_shoulder):
            self.angles['neck_h'] = self._get_neck_angle(
                model.nose[:3], model.left_shoulder[:3],
                model.right_shoulder[:3])
        # command to send
        cmd: list = ['I', [0, self.angles['neck_h']], 0]
        ardSerial.send(self.goodPorts, cmd)

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

    def _get_neck_angle(self, nose: np.array, left_shoulder: np.array,
                        right_shoulder: np.array) -> double:
        """
        Calculate neck angle
        Inputs:
          nose: nose position
          left_shoulder: left shoulder position
          right_shoulder: right shoulder position
        """
        # calculate neck angle
        shoulder_mid = (model.left_shoulder + model.right_shoulder) / 2
        mid_nose = model.nose - shoulder_mid
        left_mid = shoulder_mid - model.left_shoulder
        plane_normal = np.cross(left_mid, mid_nose)
        front_dir = np.cross(plane_normal, left_mid)
        angle = self._vec_angle(front_dir, mid_nose)
        # determine direction
        axis = 1
        if np.inner(np.cross(mid_nose, front_dir), plane_normal) > 0:
            axis = -1

        return dir * angle

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
