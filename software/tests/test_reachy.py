import unittest

from reachy import parts, Reachy

from mockup import mock_luos_io

mock_luos_io()


class ReachyTestCase(unittest.TestCase):
    def setUp(self):
        self.reachy = Reachy(
            left_arm=parts.LeftArm(io='', hand='force_gripper'),
            right_arm=parts.RightArm(io='', hand='force_gripper'),
        )

    def test_motors(self):
        self.assertEqual(len(self.reachy.left_arm.motors), 8)
        self.assertEqual(len(self.reachy.left_arm.hand.motors), 4)
        self.assertEqual(len(self.reachy.right_arm.motors), 8)
        self.assertEqual(len(self.reachy.left_arm.hand.motors), 4)
        self.assertEqual(len(self.reachy.motors), 16)
