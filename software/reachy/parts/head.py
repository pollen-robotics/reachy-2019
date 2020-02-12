"""Head part module."""

import numpy as np

from collections import OrderedDict
from scipy.spatial.transform import Rotation as R

from .part import ReachyPart
from ..io import SharedLuosIO


# FIXME: this should be defined elsewhere
def rot(axis, deg):
    """Compute 3D rotation matrix given euler rotation."""
    return R.from_euler(axis, np.deg2rad(deg)).as_dcm()


class Head(ReachyPart):
    """Head part.

    Composed of an orbita actuator as neck, two controlled antennas and one camera.
    """

    orbita_config = {
        'Pc_z': [0, 0, 25],
        'Cp_z': [0, 0, 0],
        'R': 36.7,
        'R0': np.dot(rot('z', 60), rot('y', 10)),
        'pid': [8, 0.04, 0],
        'reduction': 77.35,
        'wheel_size': 62,
        'encoder_res': 3,
    }

    dxl_motors = OrderedDict([
        ('left_antenna', {
            'id': 31, 'offset': 0.0, 'orientation': 'direct',
        }),
        ('right_antenna', {
            'id': 30, 'offset': 0.0, 'orientation': 'direct',
        }),
    ])

    def __init__(self, camera_id, luos_port):
        """Create new Head part.

        Args:
            camera_id (int): index of the camera
            luos_port (str): serial port where the Luos modules are attached
        """
        ReachyPart.__init__(self, name='head')

        self.luos_io = SharedLuosIO.with_gate('r_head', luos_port)

        self.neck = self.create_orbita_actuator('neck', self.luos_io, Head.orbita_config)

        # self.antenna_io = SharedLuosIO.with_gate('r_antenna', luos_port)
        # self.attach_dxl_motors(self.antenna_io, Head.dxl_motors)

        self.attach_dxl_motors(self.luos_io, Head.dxl_motors)

        # We import vision here to avoid OpenCV ImportError issue
        # if we are not using the Head part.
        from ..utils.vision import BackgroundVideoCapture
        self.cap = BackgroundVideoCapture(camera_id)

    def teardown(self):
        """Clean and close head part."""
        self.luos_io.close()
        self.cap.close()

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
        self.neck.orient(q, duration=duration, wait=wait)

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

    def homing(self):
        """Launch neck homing procedure."""
        self.neck.homing()

    def get_image(self):
        """Get lat grabbed image from the camera."""
        _, img = self.cap.read()
        return img
