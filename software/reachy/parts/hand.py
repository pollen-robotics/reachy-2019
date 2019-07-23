from .part import ReachyPart
from ..io import SharedLuosIO


class Hand(ReachyPart):
    def __init__(self):
        ReachyPart.__init__(self, name='hand')


class GripperHand(Hand):
    dxl_motors = {
        'wrist_pitch': 15,
        'wrist_roll': 16,
        'gripper': 19,
    }

    def __init__(self, luos_port):
        Hand.__init__(self)

        self.luos_io = SharedLuosIO(luos_port)
        self.attach_dxl_motors(self.luos_io, GripperHand.dxl_motors)
