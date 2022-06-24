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
        # start control loop
        self.control_t = threading.Thread(target=self.control_loop)
        self.mtx = threading.Lock()
        self.control_t.start()

    def control_loop(self):
        while True:
            with self.mtx:
                cmd: list = [
                    'I',
                    [
                        0, self.angles['neck_h'], 8, self.angles['shoulder_l'],
                        9, self.angles['shoulder_r'], 12,
                        self.angles['elbow_l'], 13, self.angles['elbow_r']
                    ], 0.02
                ]

            ardSerial.send(self.goodPorts, cmd)

    def control_cat(self, model: Model):
        """
        control opencat based on human pose
        Input:
          model: Model sent as message
        """
        # update neck angle
        with self.mtx:
            if self._check_thresh(model.thr, model.nose[3],
                                  model.left_shoulder[3],
                                  model.right_shoulder[3]):
                self.angles['neck_h'] = self._get_neck_angle(
                    model.left_shoulder[:3], model.right_shoulder[:3],
                    model.nose[:3])

            # update shoulder l
            if self._check_thresh(model.thr, model.left_shoulder[3],
                                  model.left_hip[3], model.left_elbow[3]):
                self.angles['shoulder_l'] = 90 - self._vec_angle(
                    (model.left_hip - model.left_shoulder)[:3],
                    (model.left_elbow - model.left_shoulder)[:3])

            # update shoulder r
            if self._check_thresh(model.thr, model.right_shoulder[3],
                                  model.right_hip[3], model.right_elbow[3]):
                self.angles['shoulder_r'] = 90 - self._vec_angle(
                    (model.right_hip - model.right_shoulder)[:3],
                    (model.right_elbow - model.right_shoulder)[:3])

            # update elbow l
            if self._check_thresh(model.thr, model.left_shoulder[3],
                                  model.left_wrist[3], model.left_elbow[3]):
                self.angles['elbow_l'] = self._vec_angle(
                    (model.left_shoulder - model.left_elbow)[:3],
                    (model.left_wrist - model.left_elbow)[:3]) - 90

            # update elbow r
            if self._check_thresh(model.thr, model.right_shoulder[3],
                                  model.right_wrist[3], model.right_elbow[3]):
                self.angles['elbow_r'] = self._vec_angle(
                    (model.right_shoulder - model.right_elbow)[:3],
                    (model.right_wrist - model.right_elbow)[:3]) - 90

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
        angle = self._vec_angle(front_dir, mid_nose)
        # determine direction
        axis = 1
        if np.inner(np.cross(mid_nose, front_dir), plane_normal) > 0:
            axis = -1

        return axis * angle

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
