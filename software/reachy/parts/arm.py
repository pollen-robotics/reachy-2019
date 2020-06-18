"""Arm part module.

Implements a Right and a Left Arm.
"""

import time
import numpy as np

from operator import attrgetter
from collections import OrderedDict
from threading import Thread, Event

from .hand import LeftEmptyHand, RightEmptyHand, LeftForceGripper, RightForceGripper, OrbitaWrist
from .part import ReachyPart


hands = {
    'force_gripper': {'left': LeftForceGripper, 'right': RightForceGripper},
    'orbita_wrist': {'left': OrbitaWrist, 'right': OrbitaWrist},
    'empty_hand': {'left': LeftEmptyHand, 'right': RightEmptyHand},
}


class Arm(ReachyPart):
    """Arm abstraction class.

    Args:
        side (str): 'right' or 'left'
        io (str): port name where the modules can be found
        dxl_motors (dict): config of the dynamixel motors composing the arm
        hand (str): name of the Hand to attach ('force_gripper', 'orbita_wrist' or it can be None if no hand are attached)

    Provides high-level access to:
        * ordered list of motors
        * forward and inverse kinematics
    """

    fans = {
        'shoulder_fan': 'shoulder_pitch',
        'elbow_fan': 'elbow_pitch',
    }
    lower_temp_threshold, upper_temp_threshold = 40, 45

    def __init__(self, side, io, dxl_motors, hand):
        """Create a new Arm part."""
        ReachyPart.__init__(self, name=f'{side}_arm', io=io)
        self.side = side

        dxl_motors = OrderedDict(dxl_motors)

        self.attach_dxl_motors(dxl_motors)

        if hand is not None and hand not in hands.keys():
            raise ValueError(f'"hand" must be one of {list(hands.keys())} or None!')

        if hand is not None:
            hand_cls = hands[hand][side]

            hand_part = hand_cls(root=self, io=io)
            self.motors += hand_part.motors
            self.hand = hand_part

            for m, conf in hand_cls.dxl_motors.items():
                dxl_motors[m] = conf

        else:
            self.hand = None

        self.attach_kinematic_chain(dxl_motors)

        self.fans = dict(Arm.fans)
        if hand is not None:
            self.fans.update(hand_cls.fans)

        for name in self.fans.keys():
            setattr(self, name, self.io.find_fan(name))

        self._monitor_temp = Event()
        self._monitor_temp_loop = None
        self.enable_temperature_monitoring()

    def teardown(self):
        """Clean up before closing."""
        if self.hand is not None:
            self.hand.teardown()
        ReachyPart.teardown(self)

    def __repr__(self):
        """Arm representation."""
        return f'<{self.side.capitalize()}Arm "motors": {self.motors} "hand": {self.hand}>'

    def forward_kinematics(self, joints_position, use_rad=False):
        """Compute the forward kinematics of the Arm.

        Args:
            joints_position (:py:class:`~numpy.ndarray`): angle joints configuration of the arm (in degrees by default)
            use_rad (bool): whether or not to use radians for joints configuration

        .. note:: the end effector will be the end of the Hand if one is attached.

        """
        joints_position = np.array(joints_position)
        if len(joints_position.shape) == 1:
            joints_position = joints_position.reshape(1, -1)

        if not use_rad:
            joints_position = np.deg2rad(joints_position)

        M = self.kin_chain.forward(joints_position)

        if joints_position.shape[0] == 1:
            M = M[0]
        return M

    def inverse_kinematics(self, target_pose, q0=None, use_rad=False, maxiter=10):
        """Approximate the inverse kinematics of the Arm.

        Args:
            target_pose (:py:class:`~numpy.ndarray`): 4x4 homogeneous pose of the target end effector pose
            q0 (:py:class:`~numpy.ndarray`): joint initial angle configurations (used for bootstraping the optimization)
            use_rad (bool): whether or not to use radians for joints configuration
            maxiter (int): maximum number of iteration to run on the optimizer

        .. note:: the end effector will be the end of the Hand if one is attached.

        """
        if q0 is None:
            q0 = [m.present_position for m in self.motors]

        q0 = np.array(q0)

        if len(q0.shape) == 1:
            q0 = q0.reshape(1, -1)

        if len(target_pose.shape) == 2:
            target_pose = target_pose.reshape(-1, 4, 4)

        if not use_rad:
            q0 = np.deg2rad(q0)

        J = self.kin_chain.inverse(target_pose, q0, maxiter=maxiter)

        if J.shape[0] == 1:
            J = J[0]

        if not use_rad:
            J = np.rad2deg(J)

        return J

    def enable_temperature_monitoring(self):
        """Enable the automatic motor cooling procedure.

        The specified motors temperature will be watched and when they reached a specific threshold, the fan will automatically be turned on.
        When the temperature goes below a lower threshold, they will turn off.
        """
        if not self._monitor_temp.is_set():
            self._monitor_temp.set()
            self._monitor_temp_loop = Thread(target=self._temperature_monitoring)
            self._monitor_temp_loop.daemon = True
            self._monitor_temp_loop.start()

    def disable_temperature_monitoring(self):
        """Disable the automatic motor cooling procedure."""
        if self._monitor_temp.is_set():
            self._monitor_temp.clear()

    def _temperature_monitoring(self):
        while self._monitor_temp.is_set():
            for fan_name, motor_name in self.fans.items():
                fan = attrgetter(fan_name)(self)
                motor = attrgetter(motor_name)(self)

                if motor.temperature is not None and motor.temperature >= self.upper_temp_threshold:
                    fan.on()
                elif motor.temperature is not None and motor.temperature <= self.lower_temp_threshold:
                    fan.off()

            time.sleep(30)


class LeftArm(Arm):
    """Left Arm part.

    Args:
        io (str): port name where the modules can be found
        hand (str): name of the :py:class:`~reachy.parts.hand.Hand` to attach ('force_gripper', 'orbita_wrist' or it can be None if no hand are attached)
    """

    dxl_motors = OrderedDict([
        ('shoulder_pitch', {
            'id': 20, 'offset': 90.0, 'orientation': 'direct',
            'angle-limits': [-60, 180],
            'link-translation': [0, 0.19, 0], 'link-rotation': [0, 1, 0]
        }),
        ('shoulder_roll', {
            'id': 21, 'offset': -90.0, 'orientation': 'indirect',
            'angle-limits': [-90, 100],
            'link-translation': [0, 0, 0], 'link-rotation': [1, 0, 0],
        }),
        ('arm_yaw', {
            'id': 22, 'offset': 0.0, 'orientation': 'indirect',
            'angle-limits': [-90, 90],
            'link-translation': [0, 0, 0], 'link-rotation': [0, 0, 1],
        }),
        ('elbow_pitch', {
            'id': 23, 'offset': 0.0, 'orientation': 'indirect',
            'angle-limits': [0, 125],
            'link-translation': [0, 0, -0.28], 'link-rotation': [0, 1, 0],
        }),
    ])

    def __init__(self, io, hand=None):
        """Create a new Left Arm part."""
        Arm.__init__(self, side='left',
                     io=io, dxl_motors=LeftArm.dxl_motors,
                     hand=hand)


class RightArm(Arm):
    """Right Arm part.

    Args:
        io (str): port name where the modules can be found
        hand (str): name of the :py:class:`~reachy.parts.hand.Hand` to attach ('force_gripper', 'orbita_wrist' or it can be None if no hand are attached)
    """

    dxl_motors = OrderedDict([
        ('shoulder_pitch', {
            'id': 10, 'offset': 90.0, 'orientation': 'indirect',
            'angle-limits': [-180, 60],
            'link-translation': [0, -0.19, 0], 'link-rotation': [0, 1, 0],
        }),
        ('shoulder_roll', {
            'id': 11, 'offset': 90.0, 'orientation': 'indirect',
            'angle-limits': [-100, 90],
            'link-translation': [0, 0, 0], 'link-rotation': [1, 0, 0],
        }),
        ('arm_yaw', {
            'id': 12, 'offset': 0.0, 'orientation': 'indirect',
            'angle-limits': [-90, 90],
            'link-translation': [0, 0, 0], 'link-rotation': [0, 0, 1],
        }),
        ('elbow_pitch', {
            'id': 13, 'offset': 0.0, 'orientation': 'indirect',
            'angle-limits': [0, 125],
            'link-translation': [0, 0, -0.28], 'link-rotation': [0, 1, 0],
        }),
    ])

    def __init__(self, io, hand=None):
        """Create a new Right Arm part."""
        Arm.__init__(self, side='right',
                     io=io, dxl_motors=RightArm.dxl_motors,
                     hand=hand)
