"""Head part module."""

import os
import pathlib
import numpy as np

from collections import OrderedDict

import reachy

from ..utils import rot
from .part import ReachyPart


class Head(ReachyPart):
    """Head part.

    Args:
        io (str): port name where the modules can be found
        default_camera (str): default camera to enable ('left' or 'right')

    Composed of an orbita actuator as neck, two controlled antennas and one camera.
    """

    orbita_config = {
        'Pc_z': [0, 0, 23],
        'Cp_z': [0, 0, 0],
        'R': 35.9,
        'R0': np.dot(rot('z', 60), rot('y', 10)),
        'hardware_zero': np.load(os.path.join(pathlib.Path(reachy.__file__).parent, 'orbita_head_hardware_zero.npy')),
    }

    dxl_motors = OrderedDict([
        ('left_antenna', {
            'id': 30, 'offset': 0.0, 'orientation': 'direct',
            'angle-limits': [-150, 150],
        }),
        ('right_antenna', {
            'id': 31, 'offset': 0.0, 'orientation': 'direct',
            'angle-limits': [-150, 150],
        }),
    ])

    def __init__(self, io, default_camera='right'):
        """Create new Head part."""
        ReachyPart.__init__(self, name='head', io=io)

        self.neck = self.create_orbita_actuator('neck', Head.orbita_config)
        self.attach_dxl_motors(Head.dxl_motors)
        self.camera = self.io.find_dual_camera(default_camera)

    def __repr__(self):
        """Head representation."""
        return f'<Head "neck": {self.neck}>'

    def teardown(self):
        """Clean and close head part."""
        self.camera.close()
        ReachyPart.teardown(self)

    def look_at(self, x, y, z, duration, wait):
        """Make the head look at a 3D point in space.

        Args:
            x (float): x coordinates in space
            y (float): y coordinates in space
            z (float): z coordinates in space
            duration (float): move duration (in seconds)
            wait (bool): whether or not to wait for the end of the motion
        """
        q = self.neck.model.find_quaternion_transform([1, 0, 0], [x, y, z])
        return self.neck.orient(q, duration=duration, wait=wait)

    @property
    def compliant(self):
        """Check if the neck is compliant."""
        return self.neck.compliant

    @compliant.setter
    def compliant(self, compliancy):
        self.neck.compliant = compliancy

    @property
    def moving_speed(self):
        """Get the current disk moving speed."""
        return self.neck.moving_speed

    @moving_speed.setter
    def moving_speed(self, speed):
        self.neck.moving_speed = speed

    @property
    def active_camera(self):
        """Get the active camera side (left or right)."""
        return self.camera.active_side

    def enable_camera(self, camera_side):
        """Enable one of the camera active (left or right)."""
        self.camera.set_active(camera_side)

    def get_image(self):
        """Get last grabbed image from the camera."""
        _, img = self.camera.read()
        return img
