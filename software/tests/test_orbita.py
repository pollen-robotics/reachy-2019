import pytest

from pyquaternion import Quaternion

from mockup import mock_luos_io

mock_luos_io()

from reachy.utils import rot  # noqa: E402
from reachy.io.luos import SharedLuosIO  # noqa: E402
from reachy.parts.motor import OrbitaActuator  # noqa: E402


def test_orbita_goto():
    luos_io = SharedLuosIO.with_gate('gate', '')
    luos_disks_motor = luos_io.find_orbita_disks()

    config = {
        'Pc_z': [0, 0, 25],
        'Cp_z': [0, 0, 0],
        'R': 36.7,
        'R0': rot('z', 60),
        'pid': [8, 0.035, 0],
        'reduction': 77.35,
        'wheel_size': 62,
        'encoder_res': 3,
    }

    orb = OrbitaActuator('', 'bob', luos_disks_motor, **config)

    with pytest.raises(TypeError):
        orb.goto(Quaternion([1, 0, 0, 0]), 1, False)

    with pytest.raises(ValueError):
        orb.goto([1, 0, 0, 0], 1, False)

    with pytest.raises(ValueError):
        orb.goto([1, 0], 1, False)
