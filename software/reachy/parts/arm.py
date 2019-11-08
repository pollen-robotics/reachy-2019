import numpy as np

from collections import OrderedDict

from .hand import ForceGripper
from .part import ReachyPart
from ..io import SharedLuosIO


hands = {
    'force_gripper': ForceGripper,
}


class Arm(ReachyPart):
    def __init__(self, side, luos_port, dxl_motors, hand):
        ReachyPart.__init__(self, name=f'{side}_arm')

        self.luos_io = SharedLuosIO.with_gate(f'r_{side}_arm', luos_port)
        self.attach_dxl_motors(self.luos_io, dxl_motors)

        self.attach_kinematic_chain(dxl_motors)

        if hand is not None and hand not in hands.keys():
            raise ValueError('"hand" must be one of {hands.keys()} or None!')

        if hand is not None:
            hand_part = hands[hand](luos_port=self.luos_io.port)
            hand_part.name = f'{self.name}.{hand}'
            self.motors += hand_part.motors
        self.hand = hand_part

    def forward_kinematics(self, joints_position, use_rad=False):
        joints_position = np.array(joints_position)
        if len(joints_position.shape) == 1:
            joints_position = joints_position.reshape(1, -1)

        if not use_rad:
            joints_position = np.deg2rad(joints_position)

        nb_arm_motors = 5

        M = self.kin_chain.forward(joints_position[:, :nb_arm_motors])

        if self.hand is not None:
            M = np.matmul(M, self.hand.kin_chain.forward(joints_position[:, nb_arm_motors:]))

        if joints_position.shape[0] == 1:
            M = M[0]
        return M


class LeftArm(Arm):
    dxl_motors = OrderedDict([
        ('shoulder_pitch', {
            'id': 20, 'offset': 90.0, 'orientation': 'direct',
            'link-translation': [0, 0.14, 0], 'link-rotation': [0, 1, 0]
        }),
        ('shoulder_roll', {
            'id': 21, 'offset': -90.0, 'orientation': 'indirect',
            'link-translation': [0, 0, 0], 'link-rotation': [1, 0, 0],
        }),
        ('arm_yaw', {
            'id': 22, 'offset': 0.0, 'orientation': 'indirect',
            'link-translation': [0, 0, 0], 'link-rotation': [0, 0, 1],
        }),
        ('elbow_pitch', {
            'id': 23, 'offset': 0.0, 'orientation': 'direct',
            'link-translation': [0, 0, -0.30745], 'link-rotation': [0, 1, 0],
        }),
        ('forearm_yaw', {
            'id': 24, 'offset': 0.0, 'orientation': 'indirect',
            'link-translation': [0, 0, 0], 'link-rotation': [0, 0, 1],
        }),
    ])

    def __init__(self, luos_port, hand=None):
        Arm.__init__(self, side='left',
                     luos_port=luos_port, dxl_motors=LeftArm.dxl_motors,
                     hand=hand)


class RightArm(Arm):
    dxl_motors = OrderedDict([
        ('shoulder_pitch', {
            'id': 10, 'offset': 90.0, 'orientation': 'indirect',
            'link-translation': [0, -0.14, 0], 'link-rotation': [0, 1, 0],
        }),
        ('shoulder_roll', {
            'id': 11, 'offset': 90.0, 'orientation': 'indirect',
            'link-translation': [0, 0, 0], 'link-rotation': [1, 0, 0],
        }),
        ('arm_yaw', {
            'id': 12, 'offset': 0.0, 'orientation': 'indirect',
            'link-translation': [0, 0, 0], 'link-rotation': [0, 0, 1],
        }),
        ('elbow_pitch', {
            'id': 13, 'offset': 0.0, 'orientation': 'indirect',
            'link-translation': [0, 0, -0.30745], 'link-rotation': [0, 1, 0],
        }),
        ('forearm_yaw', {
            'id': 14, 'offset': 0.0, 'orientation': 'indirect',
            'link-translation': [0, 0, 0], 'link-rotation': [0, 0, 1],
        }),
    ])

    def __init__(self, luos_port, hand=None):
        Arm.__init__(self, side='right',
                     luos_port=luos_port, dxl_motors=RightArm.dxl_motors,
                     hand=hand)
