import numpy as np


class Pose:
    def __init__(self, x, y, z, roll, pitch, yaw):
        self.pose = np.array([x, y, z, roll, pitch, yaw])

    def __str__(self):
        return f"X: {self.pose[0]:5.2f}, Y: {self.pose[1]:5.2f}, Z: {self.pose[2]:5.2f}"

    def coordinates(self) -> np.ndarray:
        return np.array([self.pose[0], self.pose[1], self.pose[2], 1])

    def rotation(self) -> np.ndarray:
        return np.array([self.pose[3], self.pose[4], self.pose[5], 1])

    def translate(self, x, y, z):
        self.pose[0] += x
        self.pose[1] += y
        self.pose[2] += z

    def rotate(self, roll, pitch, yaw):
        self.pose[3] += roll
        self.pose[4] += pitch
        self.pose[5] += yaw
