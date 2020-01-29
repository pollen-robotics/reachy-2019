import numpy as np

from collections import OrderedDict
from scipy.spatial.transform import Rotation as R

from .part import ReachyPart
from ..io import SharedLuosIO


def rot(axis, deg):
    return R.from_euler(axis, np.deg2rad(deg)).as_matrix()


class Head(ReachyPart):
    orbita_config = {
        'Pc_z': [0, 0, 25],
        'Cp_z': [0, 0, 0],
        'R': 36.7,
        'R0': np.dot(rot('z', 60), rot('y', 10)),
        'pid': [10, 0.04, 90],
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
        ReachyPart.__init__(self, name='head')

        self.luos_io = SharedLuosIO.with_gate('r_head', luos_port)

        self.neck = self.create_orbita_actuator('neck', self.luos_io, Head.orbita_config)
        self.attach_dxl_motors(self.luos_io, Head.dxl_motors)

        # We import vision here to avoid OpenCV ImportError issue
        # if we are not using the Head part.
        from ..utils.vision import BackgroundVideoCapture
        self.cap = BackgroundVideoCapture(camera_id)

    def teardown(self):
        self.luos_io.close()
        self.cap.close()

    def look_at(self, x, y, z, duration=-1, wait=False):
        q = self.neck.model.find_quaternion_transform([1, 0, 0], [x, y, z])
        self.neck.orient(q, duration=duration, wait=wait)

    @property
    def compliant(self):
        return self.neck.compliant

    @compliant.setter
    def compliant(self, compliancy):
        self.neck.compliant = compliancy

    @property
    def moving_speed(self):
        return self.neck.moving_speed

    @moving_speed.setter
    def moving_speed(self, speed):
        self.neck.moving_speed = speed

    def homing(self):
        self.neck.homing()

    def get_image(self):
        _, img = self.cap.read()
        return img
