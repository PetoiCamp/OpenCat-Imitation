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
        self.goodPorts = {}
        ardSerial.connectPort(self.goodPorts)
        self.t = threading.Thread(target=ardSerial.keepCheckingPort,
                                  args=(self.goodPorts, ))
        self.t.start()
        ardSerial.send(self.goodPorts, ['kcalib', 1])
        ardSerial.send(self.goodPorts,
                       ['I', [0, 0, 10, -52, 11, -52, 14, 83, 15, 83], 1])
        ardSerial.send(self.goodPorts,
                       ['I', [0, 0, 10, -15, 11, -15, 14, 83, 15, 83], 1])

    def control_cat(self, model: Model):
        """
        control opencat based on human pose
        """
        # calculate neck angle
        shoulder_mid = (model.left_shoulder + model.right_shoulder) / 2
        mid_nose = model.nose - shoulder_mid
        left_mid = shoulder_mid - model.left_shoulder
        plane_normal = np.cross(left_mid, mid_nose)
        front_dir = np.cross(plane_normal, left_mid)
        angle = self._vec_angle(front_dir, mid_nose)
        dir = 1
        if np.inner(np.cross(mid_nose, front_dir), plane_normal) > 0:
            dir = -1
        # command to send
        cmd: list = ['I', [0, angle * dir], 0]
        ardSerial.send(self.goodPorts, cmd)

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
