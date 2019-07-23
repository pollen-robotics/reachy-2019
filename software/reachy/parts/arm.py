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
        self.hand = hand


class LeftArm(Arm):
    dxl_motors = {
        'shoulder_pitch': 20,
        'shoulder_roll': 21,
        'arm_yaw': 22,
        'forearm_yaw': 23,
        'wrist_pitch': 24,
        'wrist_roll': 25
    }

    def __init__(self, luos_port, hand=None):
        Arm.__init__(self, side='left',
                     luos_port=luos_port, dxl_motors=LeftArm.dxl_motors,
                     hand=hand)


class RightArm(Arm):
    dxl_motors = {
        'shoulder_pitch': 10,
        'shoulder_roll': 11,
        'arm_yaw': 12,
        'forearm_yaw': 13,
        'wrist_pitch': 14,
        'wrist_roll': 15
    }

    def __init__(self, luos_port, hand=None):
        Arm.__init__(self, side='right',
                     luos_port=luos_port, dxl_motors=RightArm.dxl_motors,
                     hand=hand)
