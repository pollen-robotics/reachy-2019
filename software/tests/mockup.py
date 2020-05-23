import reachy

from unittest.mock import MagicMock

from pyluos import DynamixelMotor


class MockLuosIO(MagicMock):
    def __getattr__(self, attr):
        if attr.startswith('dxl_'):
            return MockDxlMotor()

        return MagicMock.__getattr__(self, attr)


class MockDxlMotor(MagicMock, DynamixelMotor):
    @property
    def temperature(self):
        return 20.0


def mock_luos_io():
    reachy.io.luos.LuosDevice = MockLuosIO()
