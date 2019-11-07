import numpy as np

from .part import ReachyPart
from ..io import SharedLuosIO


class Head(ReachyPart):
    orbital_config = {
        'Pc_z': [0, 0, 93],
        'Cp_z': [0, 0, 73],
        'R': 33.5,
        'R0': [
            [np.cos(np.deg2rad(60)), -np.sin(np.deg2rad(60)), 0],
            [np.sin(np.deg2rad(60)),  np.cos(np.deg2rad(60)), 0],
            [0, 0, 1],
        ],
        'pid': [10, 0, 90],
        'reduction': 77.35,
        'wheel_size': 62,
        'encoder_res': 5,
    }

    def __init__(self, camera_id, luos_port):
        ReachyPart.__init__(self, name='head')

        self.luos_io = SharedLuosIO(luos_port)
        self.attach_orbita_actuator('neck', self.luos_io, Head.orbital_config)

        # We import vision here to avoid OpenCV ImportError issue
        # if we are not using the Head part.
        from ..utils.vision import BackgroundVideoCapture
        self.cap = BackgroundVideoCapture(camera_id)

    def close(self):
        self.cap.close()

    def look_at(self, x, y, z):
        self.neck.model.reset_last_angles()

        q = self.neck.find_quaternion_transform([1, 0, 0], [x, y, z])
        self.neck.orient(q)
