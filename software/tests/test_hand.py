import unittest

from reachy import parts

from mockup import mock_luos_io

mock_luos_io()


class HandTestCase(unittest.TestCase):
    def setUp(self):
        self.gripper = parts.GripperHand(
            luos_port='',
        )

    def test_gripper_motors(self):
        self.assertEqual(len(self.gripper.motors), 2)
