"""Kinematic forward and inverse utility module.

* Allow the creation of kinematic chain (following a simplified DH notation)
* Compute the forward kinematic
* Provide optimization via scipy for inverse approximation
"""

import numpy as np

from scipy.spatial.transform import Rotation
from scipy.optimize import minimize


class Link(object):
    """Simplified Link following DH notation.

    Args:
        translation (:py:class:`~numpy.ndarray`): 3d translation from the previous link
        rotation (:py:class:`~numpy.ndarray`): 3d euler rotation (xyz) from the previous link
    """

    def __init__(self, translation, rotation):
        """Create a new link."""
        self.T = translation_matrix(translation)
        self.rotation = np.array(rotation).reshape(1, 3)

    def transformation_matrix(self, theta):
        """Compute local transformation matrix for given angle.

        Args:
            theta (float): joint rotation angle (in radians)
        """
        R = np.zeros((theta.shape[0], 4, 4))
        theta = theta.reshape(1, -1)

        R[:, :3, :3] = Rotation.from_rotvec(np.dot(theta.T, self.rotation)).as_dcm()
        R[:, 3, 3] = 1
        return np.matmul(self.T, R)


class Chain(object):
    """Chain of Link forming a kinematic chain.
    
    Args:
        links (list): list of :py:class:`Link`
    """

    def __init__(self, links):
        """Create the chain given the links."""
        self.links = links

    def forward(self, joints):
        """
        Compute forward kinematics of the chain given joints configurations.

        Args:
            joints (:py:class:`~numpy.ndarray`): N*J array joint rotation angle for each link (in radians)

        Returns:
            :py:class:`~numpy.ndarray`: 4x4 homogeneous matrix pose of the end effector

        .. warning:: this is a vectorized version of the forward!
        """
        M = np.eye(4)

        for l, theta in zip(self.links, joints.T):
            M = np.matmul(M, l.transformation_matrix(theta))

        return M

    def inverse(self, poses, q0s):
        """
        Approximate the inverse kinematics of the chain given end pose.

        Args:
            poses (:py:class:`~numpy.ndarray`): N*4*4 homogeneous matrix poses for the end effector
            q0s (:py:class:`~numpy.ndarray`): N*J initial joint configuration used to bootstrap the optimization

        .. warning:: this is a vectorized version of the forward!
        """
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
    """Create homogenous matrix given a 3D translation vector."""
    M = np.eye(4)
    M[:3, 3] = translation
    return M


def position_dist(P, Q):
    """Compute euclidian distance between two 3D position."""
    return np.linalg.norm(P - Q)


def rotation_dist(P, Q):
    """Compute rotation distance between two 3D rotation."""
    R = np.matmul(P, Q.T)

    A = (np.trace(R) - 1) / 2
    A = np.clip(A, -1, 1)

    theta = np.arccos(A)
    return theta


def pose_dist(M1, M2, threshold=-1):
    """Compute distance between two pose.

    The distance is defined as the sum of the position distance and the rotation distance where 1° of error is equal to 1mm error distance.
    """
    P1, R1 = M1[:3, 3], M1[:3, :3]
    P2, R2 = M2[:3, 3], M2[:3, :3]

    PD = position_dist(P1, P2)
    RD = rotation_dist(R1, R2)

    # meaning 1mm pos error ~ 1° rot error
    E = PD + np.rad2deg(RD) * 1e-3

    return E if E > threshold else 0
