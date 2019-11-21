import time
import numpy as np

from reachy.trajectory.interpolation import MinimumJerk

from .part import ReachyPart
from ..io import SharedLuosIO


class Head(ReachyPart):
    orbital_config = {
        'Pc_z': [0, 0, 25],
        'Cp_z': [0, 0, 0],
        'R': 36.7,
        'R0': [
            [np.cos(np.deg2rad(60)), -np.sin(np.deg2rad(60)), 0],
            [np.sin(np.deg2rad(60)),  np.cos(np.deg2rad(60)), 0],
            [0, 0, 1],
        ],
        'pid': [10, 0.04, 90],
        'reduction': 77.35,
        'wheel_size': 62,
        'encoder_res': 3,
    }

    def __init__(self, camera_id, luos_port):
        ReachyPart.__init__(self, name='head')

        self.luos_io = SharedLuosIO.with_gate('r_head', luos_port)
        self.neck = self.create_orbita_actuator('neck', self.luos_io, Head.orbital_config)

        # We import vision here to avoid OpenCV ImportError issue
        # if we are not using the Head part.
        from ..utils.vision import BackgroundVideoCapture
        self.cap = BackgroundVideoCapture(camera_id)

    def teardown(self):
        self.cap.close()

    def look_at(self, x, y, z):
        self.neck.model.reset_last_angles()

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

        self.look_at(1, 0, 0)
