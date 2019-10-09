from .part import ReachyPart
from ..io import SharedLuosIO


class Head(ReachyPart):
    orbital_config = {
        'Pc_z': [0, 0, 117.4],
        'Cp_z': [0, 0, 84.256],
        'R': 51.56,
    }

    def __init__(self, camera_id, luos_port):
        ReachyPart.__init__(self, name='head')

        self.luos_io = SharedLuosIO(luos_port)
        self.attach_orbita_actuator('neck', self.luos_io, Head.orbital_config)

        # We import vision here to avoid OpenCV ImportError issue
        # if we are not using the Head part.
        from ..utils.vision import BackgroundVideoCapture
        self.cap = BackgroundVideoCapture(camera_id)
