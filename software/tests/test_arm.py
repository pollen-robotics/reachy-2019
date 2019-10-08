import unittest
import numpy as np

from reachy import parts

from mockup import mock_luos_io

mock_luos_io()


class ArmTestCase(unittest.TestCase):
    def setUp(self):
        self.left_arm = parts.LeftArm(
            luos_port='',
            hand=None,
        )

        self.right_arm = parts.RightArm(
            luos_port='',
            hand=None,
        )

    def test_motors(self):
        self.assertEqual(len(self.left_arm.motors), 5)
        self.assertEqual(len(self.right_arm.motors), 5)

    def test_zero_forward(self):
        N = np.random.randint(2, 100)

        J0 = np.zeros((N, 5))
        lm0 = np.array((
            (1, 0, 0, 0),
            (0, 1, 0, 0.14),
            (0, 0, 1, -0.30745),
            (0, 0, 0, 1),
        ))
        rm0 = np.array((
            (1, 0, 0, 0),
            (0, 1, 0, -0.14),
            (0, 0, 1, -0.30745),
            (0, 0, 0, 1),
        ))

        for arm, m0 in zip(
            (self.left_arm, self.right_arm),
            (lm0, rm0)
        ):
            M0 = np.zeros((N, 4, 4))
            for i in range(N):
                M0[i] = m0

            self.assertAlmostEqual(
                np.linalg.norm(arm.forward_kinematics(J0[0]) - M0[0]),
                0,
            )

            self.assertAlmostEqual(
                np.linalg.norm(arm.forward_kinematics(J0) - M0),
                0,
            )
