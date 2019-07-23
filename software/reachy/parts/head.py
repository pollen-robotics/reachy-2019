from .part import ReachyPart
from ..io import SharedLuosIO


class Head(ReachyPart):
    dxl_motors = {
        'pan': 8,
        'tilt': 5,
    }

    def __init__(self, camera_id, luos_port):
        ReachyPart.__init__(self, name='head')

        self.luos_io = SharedLuosIO(luos_port)
        self.attach_dxl_motors(self.luos_io, Head.dxl_motors)

        # We import vision here to avoid OpenCV ImportError issue
        # if we are not using the Head part.
        from ..utils.vision import BackgroundVideoCapture
        self.cap = BackgroundVideoCapture(camera_id)
