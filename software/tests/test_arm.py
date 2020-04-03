import unittest
import numpy as np

from reachy import parts

from mockup import mock_luos_io

mock_luos_io()


class ArmTestCase(unittest.TestCase):
    def setUp(self):
        self.left_arm = parts.LeftArm(
            io='',
            hand=None,
        )
        self.left_arm_with_gripper = parts.LeftArm(
            io='',
            hand='force_gripper',
        )
        self.right_arm = parts.RightArm(
            io='',
            hand=None,
        )
        self.right_arm_with_gripper = parts.RightArm(
            io='',
            hand='force_gripper',
        )
        self.right_arm_with_wrist = parts.RightArm(
            io='',
            hand='orbita_wrist',
        )

    def test_motors(self):
        self.assertEqual(len(self.left_arm.motors), 4)
        self.assertEqual(len(self.left_arm_with_gripper.motors), 8)
        self.assertEqual(len(self.right_arm.motors), 4)
        self.assertEqual(len(self.right_arm_with_gripper.motors), 8)
        self.assertEqual(len(self.right_arm_with_wrist.motors), 4)

    def test_zero_forward_no_hand(self):
        N = np.random.randint(2, 100)

        J0 = np.zeros((N, len(self.left_arm.motors)))
        lm0 = np.array((
            (1, 0, 0, 0),
            (0, 1, 0, 0.19),
            (0, 0, 1, -0.30745),
            (0, 0, 0, 1),
        ))
        rm0 = np.array((
            (1, 0, 0, 0),
            (0, 1, 0, -0.19),
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

    def test_zero_forward_gripper(self):
        N = np.random.randint(2, 100)

        J0 = np.zeros((N, len(self.right_arm_with_gripper.motors)))
        lm0 = np.array((
            (1, 0, 0, 0),
            (0, 1, 0, 0.1715),
            (0, 0, 1, -0.62413),
            (0, 0, 0, 1),
        ))
        rm0 = np.array((
            (1, 0, 0, 0),
            (0, 1, 0, -0.2085),
            (0, 0, 1, -0.62413),
            (0, 0, 0, 1),
        ))

        for arm, m0 in zip(
            (self.left_arm_with_gripper, self.right_arm_with_gripper),
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

    def test_inverse_kinematics(self):
        for arm in [self.left_arm, self.right_arm, self.left_arm_with_gripper, self.right_arm_with_gripper]:
            N = np.random.randint(1, 3)

            J0 = np.zeros((N, len(arm.motors)))
            M0 = arm.forward_kinematics(J0)

            noise_amp = np.random.rand() * 10
            joint_noise = np.random.rand(*J0.shape) * noise_amp - (noise_amp / 2)
            Q0 = J0 + joint_noise

            J1 = arm.inverse_kinematics(M0, Q0, maxiter=100)
            M1 = arm.forward_kinematics(J1)

            if N > 1:
                err = np.linalg.norm(M0 - M1, axis=(0, 1))
            else:
                err = np.linalg.norm(M0 - M1)

            assert np.all(err < 1e-3)

    def test_wrist(self):
        assert hasattr(self.right_arm_with_wrist.hand, 'wrist')
