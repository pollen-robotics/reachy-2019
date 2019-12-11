import time
import numpy as np

from collections import OrderedDict
from scipy.spatial.transform import Rotation as R

from .part import ReachyPart
from ..io import SharedLuosIO


def rot(axis, deg):
    return R.from_euler(axis, np.deg2rad(deg)).as_dcm()


class Hand(ReachyPart):
    def __init__(self):
        ReachyPart.__init__(self, name='hand')


class ForceGripper(Hand):
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
        self.attach_dxl_motors(self.luos_io, ForceGripper.dxl_motors)

        self._load_sensor = self.luos_io.find_module('force_gripper')
        self._load_sensor.offset = 4
        self._load_sensor.scale = 10000

    def open(self, end_pos=-30, duration=1):
        self.gripper.goto(
            goal_position=end_pos,
            duration=duration,
            wait=True,
            interpolation_mode='minjerk',
        )

    def close(self, end_pos=30, duration=1, target_grip_force=100):
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

        self.gripper.goal_position = self.gripper.present_position
        time.sleep(0.25)

        while self.grip_force > target_grip_force + 30:
            self.gripper.goal_position -= 0.1
            time.sleep(0.02)

    @property
    def grip_force(self):
        return self._load_sensor.load


class OrbitaWrist(Hand):
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

    def __init__(self, luos_port):
        Hand.__init__(self)

        self.luos_io = SharedLuosIO(luos_port)
        self.wrist = self.create_orbita_actuator('wrist', self.luos_io, OrbitaWrist.orbita_config)
