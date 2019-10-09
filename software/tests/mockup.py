import reachy

from unittest.mock import MagicMock

from pyluos import DynamixelMotor


class MockLuosIO(MagicMock):
    def __getattr__(self, attr):
        if attr.startswith('dxl_'):
            return MockDxlMotor()

        return MagicMock.__getattr__(self, attr)


class MockDxlMotor(MagicMock, DynamixelMotor):
    pass


def mock_luos_io():
    reachy.io.luos.LuosIO = MockLuosIO()
