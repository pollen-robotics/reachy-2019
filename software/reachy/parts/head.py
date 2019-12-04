import time
import numpy as np

from collections import OrderedDict
from scipy.spatial.transform import Rotation as R

from reachy.trajectory.interpolation import MinimumJerk

from .part import ReachyPart
from ..io import SharedLuosIO


def rot(axis, deg):
    return R.from_euler(axis, np.deg2rad(deg)).as_dcm()


class Head(ReachyPart):
    orbital_config = {
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
            'id': 30, 'offset': -10.0, 'orientation': 'direct',
        }),
        ('right_antenna', {
            'id': 31, 'offset': 30.0, 'orientation': 'direct',
        }),
    ])

    def __init__(self, camera_id, luos_port):
        ReachyPart.__init__(self, name='head')

        self.luos_io = SharedLuosIO.with_gate('r_head', luos_port)

        self.neck = self.create_orbita_actuator('neck', self.luos_io, Head.orbital_config)
        self.attach_dxl_motors(self.luos_io, Head.dxl_motors)

        # We import vision here to avoid OpenCV ImportError issue
        # if we are not using the Head part.
        from ..utils.vision import BackgroundVideoCapture
        self.cap = BackgroundVideoCapture(camera_id)

    def teardown(self):
        self.cap.close()

    def look_at(self, x, y, z):
        q = self.neck.find_quaternion_transform([1, 0, 0], [x, y, z])
        self.neck.orient(q)

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

    def homing(self, dur=2):
        self.compliant = True
        time.sleep(0.1)

        for d in self.neck.disks:
            d.setToZero()

        time.sleep(0.1)

        self.compliant = False
        time.sleep(0.1)

        for d in self.neck.disks:
            d.sampling_freq = 100

        traj = MinimumJerk(0, -180, dur).interpolate(np.linspace(0, dur, dur * 100))

        for d in self.neck.disks:
            d.play()
            d.target_rot_position = traj

        time.sleep(0.5)

        while np.any(np.array([d.rot_speed for d in self.neck.disks]) < 0):
            time.sleep(0.01)

        for d in self.neck.disks:
            d.stop()
            d.setToZero()

        time.sleep(0.2)

        for d in self.neck.disks:
            d.target_rot_position = 101.23

        time.sleep(2)

        for d in self.neck.disks:
            d.setToZero()

        time.sleep(0.2)
        self.neck.model.reset_last_angles()

        self.look_at(1, 0, 0)
        time.sleep(1)

    def get_image(self):
        _, img = self.cap.read()
        return img
