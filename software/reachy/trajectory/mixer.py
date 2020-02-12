"""Utility module to automatically merge trajectories.

You should always check the result of the combination of your trajectories before applying it to the robot.
Indeed, the code below is by no mean safe. Artefacts could notably be introduced in the process.

"""
import numpy as np


def sigmoid(x):
    """Compute sigmoid function."""
    return 1 / (1 + np.exp(-x))


def norm_sigmoid(n, r):
    """Compute a normalized sigmoid function with specified range.

    Args:
        n (int): number of samples to used
        r (float): X range [-r, r] where to compute the sigmoid
    """
    y = sigmoid(np.linspace(-r, r, n))
    y = y - y.min()
    y = y / y.max()
    return y


def _combine(m1, m2, overlap, r):
    n1, y = m1.shape
    n2 = m2.shape[0]

    w = int(np.round((n1 + n2) * overlap))
    n = n1 + n2 - w

    M1 = np.zeros((n, y))
    M1[:n1] = m1

    M2 = np.zeros((n, y))
    M2[-n2:] = m2

    a = n - n2
    b = n1

    W = np.zeros((n, 1))
    W[:a] = 1
    W[a:b, 0] = 1 - norm_sigmoid(b - a, r)
    W[b:] = 0

    M = ((M1 * W) + (M2 * (1 - W)))
    return M


def traj_as_array(traj):
    """Transform a trajectory dict into a numpy 2D array.

    Args:
        traj (dict): {motor_name: list of positions}
    """
    return np.array(list(traj.values())).T


def combine(*trajs, overlap=0.1, r=5):
    """Combine multiple trajectory into a single one.

    Args:
        trajs (list): list of trajectories (as dict)
        overlap (float): overlap recovery percentage for combining trajectories
        r (int): sigmoid slope factor used to merge two trajectories
    """
    motors = set([tuple(t.keys()) for t in trajs])
    if len(motors) != 1:
        raise ValueError('All trajs must use the same motors!')
    motors = motors.pop()

    moves = [traj_as_array(t) for t in trajs]

    m1 = moves[0]

    for m2 in moves[1:]:
        m1 = _combine(m1, m2, overlap, r)

    return {
        m: m1[:, i]
        for i, m in enumerate(motors)
    }
