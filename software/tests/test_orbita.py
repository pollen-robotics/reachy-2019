import pytest
import numpy as np

from pyquaternion import Quaternion

from mockup import mock_luos_io

mock_luos_io()

from reachy import parts  # noqa: E402
from reachy.utils import rot  # noqa: E402
from reachy.io.luos import SharedLuosIO  # noqa: E402
from reachy.parts.motor import OrbitaActuator  # noqa: E402


def test_head_mockup_zero():
    assert parts.Head.orbita_config['hardware_zero'] == [0, 0, 0]

    from reachy.conf import settings

    random_zero = list(np.random.rand(3))
    for i in range(len(random_zero)):
        settings.hardware.ORBITA_NECK_HARDWARE_ZERO[i] = random_zero[i]
    assert parts.Head.orbita_config['hardware_zero'] == random_zero

    for i in range(len(random_zero)):
        settings.hardware.ORBITA_NECK_HARDWARE_ZERO[i] = 0


def test_orbita_goto():
    luos_io = SharedLuosIO.with_gate('gate', '')
    luos_disks_motor = luos_io.find_orbita_disks()

    config = {
        'Pc_z': [0, 0, 25],
        'Cp_z': [0, 0, 0],
        'R': 36.7,
        'R0': rot('z', 60),
        'hardware_zero': np.zeros(3),
    }

    orb = OrbitaActuator('', 'bob', luos_disks_motor, **config)

    with pytest.raises(TypeError):
        orb.goto(Quaternion([1, 0, 0, 0]), 1, False)

    with pytest.raises(ValueError):
        orb.goto([1, 0, 0, 0], 1, False)

    with pytest.raises(ValueError):
        orb.goto([1, 0], 1, False)
