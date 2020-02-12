"""Hand part modules.

Define different hand parts:
* a ForceGripper
* a OrbitaWrist
"""

import time
import numpy as np

from collections import OrderedDict
from scipy.spatial.transform import Rotation as R

from .part import ReachyPart
from ..io import SharedLuosIO


# FIXME: this should be defined elsewhere
def rot(axis, deg):
    """Compute 3D rotation matrix given euler rotation."""
    return R.from_euler(axis, np.deg2rad(deg)).as_dcm()


class Hand(ReachyPart):
    """Hand abstraction part."""

    def __init__(self, side):
        """Create hand part.

        Args:
            side (str): which side the hand is attached to ('left' or 'right')
        """
        ReachyPart.__init__(self, name='hand')

        self.side = side


class ForceGripper(Hand):
    """Force Gripper Part.

    Composed of three dynamixel motors and a force sensor for gripping pressure.
    """

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

    def __init__(self, luos_port, side):
        """Create a new Force Gripper Hand.

        Args:
            luos_port (str): Luos port where the modules are attached
            side (str): which side the part is attached to ('left' or 'right')
        """
        Hand.__init__(self, side)

        self.luos_io = SharedLuosIO(luos_port)

        dxl_motors = OrderedDict({
            name: dict(conf)
            for name, conf in ForceGripper.dxl_motors.items()
        })

        if self.side == 'left':
            for name, conf in dxl_motors.items():
                conf['id'] += 10

        self.attach_dxl_motors(self.luos_io, dxl_motors)

        self._load_sensor = self.luos_io.find_module('force_gripper')
        self._load_sensor.offset = 4
        self._load_sensor.scale = 10000

    def open(self, end_pos=-30, duration=1):
        """Open the gripper.

        Args:
            end_pos (float): open end position (in degrees)
            duration (float): open move duration (in seconds)
        """
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
        """
        motion = self.gripper.goto(
            goal_position=end_pos,
            duration=duration,
            wait=False,
            interpolation_mode='minjerk',
        )
        while self.grip_force < target_grip_force and self.gripper.present_position < 15:
            time.sleep(0.01)

        motion.stop()
        time.sleep(0.1)

        self.gripper.goal_position = self.gripper.present_position - 2
        time.sleep(0.5)

        # while self.grip_force > target_grip_force + 30:
        #     self.gripper.goal_position -= 0.1
        #     time.sleep(0.02)

    @property
    def grip_force(self):
        """Get current grip force."""
        return self._load_sensor.load


class OrbitaWrist(Hand):
    """Orbita Wrist hand.

    A 3dof Orbita Wrist at the end of the arm.
    """

    dxl_motors = {}
    orbita_config = {
        'Pc_z': [0, 0, 25],
        'Cp_z': [0, 0, 0],
        'R': 36.7,
        'R0': rot('z', 60),
        'pid': [10, 0.04, 90],
        'reduction': 77.35,
        'wheel_size': 62,
        'encoder_res': 3,
    }

    def __init__(self, luos_port, side):
        """Create a new OrbitaWrist Hand.

        Args:
            luos_port (str): Luos port where the modules are attached
            side (str): which side the part is attached to ('left' or 'right')
        """
        Hand.__init__(self, side)

        self.luos_io = SharedLuosIO(luos_port)
        self.wrist = self.create_orbita_actuator('wrist', self.luos_io, OrbitaWrist.orbita_config)

    def homing(self):
        """Launch Wrist homing procedure."""
        self.wrist.homing()
