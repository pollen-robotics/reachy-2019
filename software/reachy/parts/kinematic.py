import numpy as np

from scipy.spatial.transform import Rotation


class Link(object):
    def __init__(self, translation, rotation):
        self.T = translation_matrix(translation)
        self.rotation = np.array(rotation)

    def transformation_matrix(self, theta):
        R = np.eye(4)
        R[:3, :3] = Rotation.from_rotvec(self.rotation * theta).as_dcm()
        return np.matmul(self.T, R)


class Chain(object):
    def __init__(self, links):
        self.links = links

    def forward(self, joints):
        M = np.eye(4)

        for l, theta in zip(self.links, joints):
            M = np.matmul(M, l.transformation_matrix(theta))

        return M


def translation_matrix(translation):
    M = np.eye(4)
    M[:3, 3] = translation
    return M
