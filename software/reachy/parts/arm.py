from .hand import Hand
from .part import ReachyPart
from ..io import SharedLuosIO


class Arm(ReachyPart):
    def __init__(self, side, luos_port, dxl_motors, hand):
        ReachyPart.__init__(self, name='{}_arm'.format(side))

        self.luos_io = SharedLuosIO(luos_port)
        self.attach_dxl_motors(self.luos_io, dxl_motors)

        if hand is not None and not isinstance(hand, Hand):
            raise ValueError('"hand" must be a Hand or None!')

        hand.name = f'{self.name}.{hand.name}'
        self.hand = hand


class LeftArm(Arm):
    dxl_motors = {
        'shoulder_pitch': {'id': 20, 'offset': 90.0, 'orientation': 'direct'},
        'shoulder_roll': {'id': 21, 'offset': -90.0, 'orientation': 'indirect'},
        'arm_yaw': {'id': 22, 'offset': 0.0, 'orientation': 'indirect'},
        'elbow_pitch': {'id': 23, 'offset': 0.0, 'orientation': 'direct'},
        'forearm_yaw': {'id': 24, 'offset': 0.0, 'orientation': 'indirect'},
    }

    def __init__(self, luos_port, hand=None):
        Arm.__init__(self, side='left',
                     luos_port=luos_port, dxl_motors=LeftArm.dxl_motors,
                     hand=hand)


class RightArm(Arm):
    dxl_motors = {
        'shoulder_pitch': {'id': 10, 'offset': 90.0, 'orientation': 'indirect'},
        'shoulder_roll': {'id': 11, 'offset': 90.0, 'orientation': 'indirect'},
        'arm_yaw': {'id': 12, 'offset': 0.0, 'orientation': 'indirect'},
        'elbow_pitch': {'id': 13, 'offset': 0.0, 'orientation': 'indirect'},
        'forearm_yaw': {'id': 14, 'offset': 0.0, 'orientation': 'indirect'},
    }

    def __init__(self, luos_port, hand=None):
        Arm.__init__(self, side='right',
                     luos_port=luos_port, dxl_motors=RightArm.dxl_motors,
                     hand=hand)
