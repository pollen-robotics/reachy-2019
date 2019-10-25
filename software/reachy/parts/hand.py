from collections import OrderedDict

from .part import ReachyPart
from ..io import SharedLuosIO


class Hand(ReachyPart):
    def __init__(self):
        ReachyPart.__init__(self, name='hand')


class GripperHand(Hand):
    dxl_motors = OrderedDict([
        ('wrist_pitch', {
            'id': 15, 'offset': 0.0, 'orientation': 'indirect',
            'link-translation': [0, 0, -0.22425], 'link-rotation': [0, 1, 0],
        }),
        ('wrist_roll', {
            'id': 16, 'offset': 0.0, 'orientation': 'indirect',
            'link-translation': [0, 0, -0.03243], 'link-rotation': [1, 0, 0],
        }),
        ('gripper', {
            'id': 17, 'offset': 0.0, 'orientation': 'direct',
            'link-translation': [0, -0.0185, -0.06], 'link-rotation': [0, 0, 0],
        }),
    ])

    def __init__(self, luos_port):
        Hand.__init__(self)

        self.luos_io = SharedLuosIO(luos_port)
        self.attach_dxl_motors(self.luos_io, GripperHand.dxl_motors)

        self.attach_kinematic_chain(GripperHand.dxl_motors)
