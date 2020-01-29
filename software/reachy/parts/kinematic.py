import numpy as np

from scipy.spatial.transform import Rotation
from scipy.optimize import minimize


class Link(object):
    def __init__(self, translation, rotation):
        self.T = translation_matrix(translation)
        self.rotation = np.array(rotation).reshape(1, 3)

    def transformation_matrix(self, theta):
        R = np.zeros((theta.shape[0], 4, 4))
        theta = theta.reshape(1, -1)

        R[:, :3, :3] = Rotation.from_rotvec(np.dot(theta.T, self.rotation)).as_matrix()
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

    def inverse(self, poses, q0s):
        return np.array([
            self._inverse(p, q0)
            for p, q0 in zip(poses, q0s)
        ])

    def _inverse(self, target, q0):
        return minimize(
            lambda j: pose_dist(self.forward(np.array(j).reshape(1, -1))[0], target),
            x0=q0,
            options={
                'maxiter': 100,
            }
        ).x


def translation_matrix(translation):
    M = np.eye(4)
    M[:3, 3] = translation
    return M


def position_dist(P, Q):
    return np.linalg.norm(P - Q)


def rotation_dist(P, Q):
    R = np.matmul(P, Q.T)

    A = (np.trace(R) - 1) / 2
    A = np.clip(A, -1, 1)

    theta = np.arccos(A)
    return theta


def pose_dist(M1, M2, threshold=-1):
    P1, R1 = M1[:3, 3], M1[:3, :3]
    P2, R2 = M2[:3, 3], M2[:3, :3]

    PD = position_dist(P1, P2)
    RD = rotation_dist(R1, R2)

    # meaning 1mm pos error ~ 1° rot error
    E = PD + np.rad2deg(RD) * 1e-3

    return E if E > threshold else 0
