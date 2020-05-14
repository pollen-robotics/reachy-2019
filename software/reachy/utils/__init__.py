"""Utility toolbox submodules."""

import numpy as np

from scipy.spatial.transform import Rotation as R


def rot(axis, deg):
    """Compute 3D rotation matrix given euler rotation."""
    return R.from_euler(axis, np.deg2rad(deg)).as_matrix()
