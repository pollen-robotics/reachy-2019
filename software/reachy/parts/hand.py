from .part import ReachyPart
from ..io import SharedLuosIO


class Hand(ReachyPart):
    def __init__(self):
        ReachyPart.__init__(self, name='hand')


class GripperHand(Hand):
    dxl_motors = {
        'wrist_pitch': {'id': 15, 'offset': 0.0, 'orientation': 'indirect'},
        'wrist_roll': {'id': 16, 'offset': 0.0, 'orientation': 'indirect'},
        'gripper': {'id': 19, 'offset': 0.0, 'orientation': 'direct'},
    }

    def __init__(self, luos_port):
        Hand.__init__(self)

        self.luos_io = SharedLuosIO(luos_port)
        self.attach_dxl_motors(self.luos_io, GripperHand.dxl_motors)
