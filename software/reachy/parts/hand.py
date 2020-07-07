"""Hand part modules.

Define different hand parts:

* a :py:class:`~reachy.parts.hand.ForceGripper`
* a :py:class:`~reachy.parts.hand.OrbitaWrist`
"""

import time
import numpy as np

from collections import OrderedDict

from ..utils import rot
from .part import ReachyPart


class Hand(ReachyPart):
    """Hand abstraction part.

    Args:
        side (str): which side the hand is attached to ('left' or 'right')
        io (str): port name where the modules can be found
    """

    fans = {}

    def __init__(self, root, io):
        """Create hand part."""
        ReachyPart.__init__(self, name=f'{root.name}.hand', io=io)

        self.side = root.side


class EmptyHand(Hand):
    """Wrist with no hand Part.

    Args:
        io (str): port name where the modules can be found
        side (str): which side the part is attached to ('left' or 'right')

    Composed of three dynamixel motors.
    """

    fans = {'wrist_fan': 'hand.wrist_pitch'}

    def __init__(self, root, io):
        """Create a new Empty Hand."""
        Hand.__init__(self, root=root, io=io)

        dxl_motors = OrderedDict({
            name: dict(conf)
            for name, conf in self.dxl_motors.items()
        })

        self.attach_dxl_motors(dxl_motors)


class LeftEmptyHand(EmptyHand):
    """Left Empty Hand Part."""

    dxl_motors = OrderedDict([
        ('forearm_yaw', {
            'id': 24, 'offset': 0.0, 'orientation': 'indirect',
            'angle-limits': [-100, 100],
            'link-translation': [0, 0, 0], 'link-rotation': [0, 0, 1],
        }),
        ('wrist_pitch', {
            'id': 25, 'offset': 0.0, 'orientation': 'indirect',
            'angle-limits': [-45, 45],
            'link-translation': [0, 0, -0.25], 'link-rotation': [0, 1, 0],
        }),
        ('wrist_roll', {
            'id': 26, 'offset': 0.0, 'orientation': 'indirect',
            'angle-limits': [-45, 45],
            'link-translation': [0, 0, -0.0325], 'link-rotation': [1, 0, 0],
        }),
    ])


class RightEmptyHand(EmptyHand):
    """Right Empty Hand Part."""

    dxl_motors = OrderedDict([
        ('forearm_yaw', {
            'id': 14, 'offset': 0.0, 'orientation': 'indirect',
            'angle-limits': [-100, 100],
            'link-translation': [0, 0, 0], 'link-rotation': [0, 0, 1],
        }),
        ('wrist_pitch', {
            'id': 15, 'offset': 0.0, 'orientation': 'indirect',
            'angle-limits': [-45, 45],
            'link-translation': [0, 0, -0.25], 'link-rotation': [0, 1, 0],
        }),
        ('wrist_roll', {
            'id': 16, 'offset': 0.0, 'orientation': 'indirect',
            'angle-limits': [-45, 45],
            'link-translation': [0, 0, -0.0325], 'link-rotation': [1, 0, 0],
        }),
    ])


class ForceGripper(Hand):
    """Force Gripper Part.

    Args:
        io (str): port name where the modules can be found
        side (str): which side the part is attached to ('left' or 'right')

    Composed of three dynamixel motors and a force sensor for gripping pressure.
    """

    fans = {'wrist_fan': 'hand.wrist_pitch'}

    def __init__(self, root, io):
        """Create a new Force Gripper Hand."""
        Hand.__init__(self, root=root, io=io)

        dxl_motors = OrderedDict({
            name: dict(conf)
            for name, conf in self.dxl_motors.items()
        })

        self.attach_dxl_motors(dxl_motors)

        self._load_sensor = self.io.find_module('force_gripper')
        self._load_sensor.offset = 4
        self._load_sensor.scale = 10000

    def __repr__(self):
        """Force gripper representation."""
        return f'<ForceGripper "grip force": {self.grip_force}>'

    def open(self, end_pos=-30, duration=1):
        """Open the gripper.

        Args:
            end_pos (float): open end position (in degrees)
            duration (float): open move duration (in seconds)
        """
        if self.side == 'left':
            end_pos = -end_pos

        self.gripper.goto(
            goal_position=end_pos,
            duration=duration,
            wait=True,
            interpolation_mode='minjerk',
        )

    def close(self, end_pos=30, duration=1, target_grip_force=100):
        """Close the gripper.

        Args:
            end_pos (float): close end position (in degrees)
            duration (float): close move duration (in seconds)
            target_grip_force (float): force threshold to stop closing the gripper

        Returns:
            bool: whether we did grasp something
        """
        release_threshold = 2

        if self.side == 'left':
            end_pos = -end_pos
            release_threshold = -release_threshold

        motion = self.gripper.goto(
            goal_position=end_pos,
            duration=duration,
            wait=False,
            interpolation_mode='minjerk',
        )

        while abs(self.grip_force) < target_grip_force and motion.is_playing:
            time.sleep(0.01)

        motion.stop()
        time.sleep(0.1)

        self.gripper.goal_position = self.gripper.present_position - release_threshold
        time.sleep(0.5)

        return abs(self.grip_force) > 0.5 * target_grip_force

    @property
    def grip_force(self):
        """Get current grip force."""
        return self._load_sensor.load


class LeftForceGripper(ForceGripper):
    """Left Force Gripper Part."""

    dxl_motors = OrderedDict([
        ('forearm_yaw', {
            'id': 24, 'offset': 0.0, 'orientation': 'indirect',
            'angle-limits': [-100, 100],
            'link-translation': [0, 0, 0], 'link-rotation': [0, 0, 1],
        }),
        ('wrist_pitch', {
            'id': 25, 'offset': 0.0, 'orientation': 'indirect',
            'angle-limits': [-45, 45],
            'link-translation': [0, 0, -0.25], 'link-rotation': [0, 1, 0],
        }),
        ('wrist_roll', {
            'id': 26, 'offset': 0.0, 'orientation': 'indirect',
            'angle-limits': [-45, 45],
            'link-translation': [0, 0, -0.0325], 'link-rotation': [1, 0, 0],
        }),
        ('gripper', {
            'id': 27, 'offset': 0.0, 'orientation': 'direct',
            'angle-limits': [-20, 69],
            'link-translation': [0, -0.01, -0.075], 'link-rotation': [0, 0, 0],
        }),
    ])


class RightForceGripper(ForceGripper):
    """Right Force Gripper Part."""

    dxl_motors = OrderedDict([
        ('forearm_yaw', {
            'id': 14, 'offset': 0.0, 'orientation': 'indirect',
            'angle-limits': [-100, 100],
            'link-translation': [0, 0, 0], 'link-rotation': [0, 0, 1],
        }),
        ('wrist_pitch', {
            'id': 15, 'offset': 0.0, 'orientation': 'indirect',
            'angle-limits': [-45, 45],
            'link-translation': [0, 0, -0.25], 'link-rotation': [0, 1, 0],
        }),
        ('wrist_roll', {
            'id': 16, 'offset': 0.0, 'orientation': 'indirect',
            'angle-limits': [-45, 45],
            'link-translation': [0, 0, -0.0325], 'link-rotation': [1, 0, 0],
        }),
        ('gripper', {
            'id': 17, 'offset': 0.0, 'orientation': 'direct',
            'angle-limits': [-69, 20],
            'link-translation': [0, -0.01, -0.075], 'link-rotation': [0, 0, 0],
        }),
    ])


class OrbitaWrist(Hand):
    """Orbita Wrist hand.

    Args:
        io (str): port name where the modules can be found
        side (str): which side the part is attached to ('left' or 'right')

    A 3dof Orbita Wrist at the end of the arm.
    """

    dxl_motors = {}
    orbita_config = {
        'Pc_z': [0, 0, 25],
        'Cp_z': [0, 0, 0],
        'R': 36.7,
        'R0': rot('z', 60),
        'hardware_zero': np.zeros(3),
    }

    def __init__(self, root, io):
        """Create a new OrbitaWrist Hand."""
        Hand.__init__(self, root=root, io=io)

        self.wrist = self.create_orbita_actuator('wrist', OrbitaWrist.orbita_config)

    def __repr__(self):
        """Orbita wrist representation."""
        return f'<OrbitaWrist "wrist": {self.wrist}>'

    def homing(self):
        """Launch Wrist homing procedure."""
        self.wrist.homing()
