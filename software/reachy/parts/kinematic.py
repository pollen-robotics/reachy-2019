import numpy as np

from scipy.spatial.transform import Rotation


class Link(object):
    def __init__(self, translation, rotation):
        self.T = translation_matrix(translation)
        self.rotation = np.array(rotation)

    def transformation_matrix(self, theta):
        R = np.zeros((theta.shape[0], 4, 4))
        R[:, :3, :3] = Rotation.from_rotvec(np.matrix(theta).T * self.rotation).as_dcm()
        R[:, 3, 3] = 1
        return np.matmul(self.T, R)


class Chain(object):
    def __init__(self, links):
        self.links = links

    def forward(self, joints):
        M = np.eye(4)

        for l, theta in zip(self.links, joints.T):
            M = np.matmul(M, l.transformation_matrix(theta))

        return M


def translation_matrix(translation):
    M = np.eye(4)
    M[:3, 3] = translation
    return M
