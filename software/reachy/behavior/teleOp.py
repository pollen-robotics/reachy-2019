"""tele-operation part module."""
import time
import numpy as np

from threading import Thread


class TeleOp(object):
    """Class to be use by tele-operation applications.

    Args:
        reachy (:py:class:`~reachy.Reachy`): robot which will follow the movements
        left_arm (reachy.parts.LeftArm): left arm part if present or None if absent
        right_arm (reachy.parts.RightArm): right arm part if present or None if absent

    Provides easy-to-use method to make reachy follow hands position, rotation and hand opening.
    """

    def __init__(self, left_arm, right_arm):
        """Create the tele-operation instance."""
        self.left_arm = left_arm
        self.right_arm = right_arm
        self.is_moving = False

    def start_moving(self, wait=0):
        """Create thread and call the follow_movements method.

        Args:
            wait : float
                time in seconds to wait before start to move
        """
        time.sleep(wait)

        self.stop_moving()

        self.move_t = Thread(target=self.follow_movements, daemon=True)
        self.is_moving = True
        self.move_t.start()

    def stop_moving(self):
        """Make reachy stop moving."""
        self.is_moving = False

        if hasattr(self, 'move_t') and self.move_t.is_alive():
            self.move_t.join()

    def follow_movements(self):
        """Move along with VR poses of both hands."""
        self.update_state(self.right_arm.io.ws.state_dict)
        is_left_hand_opened = self.left_hand_command
        is_right_hand_opened = self.right_hand_command
        while self.is_moving:
            self.update_state(self.right_arm.io.ws.state_dict)

            is_left_hand_opened = self.move_arm(self.left_arm, self.left_hand_command, is_left_hand_opened)
            is_right_hand_opened = self.move_arm(self.right_arm, self.right_hand_command, is_right_hand_opened)

    def move_arm(self, arm, hand_command, is_hand_opened):
        """Move one arm according to the pose given by update_state.

        Args:
            arm (reachy.parts.arm): the arm to move
            hand_command (bool): true if the hand is supposed to be opened/closed according to the state received by the webSocket, false otherwise
            is_hand_opened (bool): the current hand's state

        The boleans arguments are used to check wether the state has changed or not. It allows us not to call the open()/close() method when
        the hand is already opened/closed.
        """
        if(hand_command != is_hand_opened):
            is_hand_opened = hand_command
            if(is_hand_opened):
                self.open_t = Thread(target=self.open_hand, args=(arm,))
                self.open_t.start()
            else:
                self.close_t = Thread(target=self.close_hand, args=(arm,))
                self.close_t.start()

        current_position = [m.present_position for m in arm.motors]

        if(arm.side == self.left_arm.side):
            targetJoint = arm.inverse_kinematics(self.leftPose, q0=current_position)
            self.goto_left_arm_joint_solution(targetJoint, duration=0.1, wait=True)
        else:
            targetJoint = arm.inverse_kinematics(self.rightPose, q0=current_position)
            self.goto_right_arm_joint_solution(targetJoint, duration=0.1, wait=True)

        return is_hand_opened

    def update_state(self, state_dict):
        """Update the fields used in the followMovements() and move_arm() mehods.

        Args:
        state_dict (dictionnary) : contains all the informations to make both hands moving.
        It is updated in the webSocket sync loop.
        """
        if('rightHand' in state_dict and 'leftHand' in state_dict):
            right_pose_dict = state_dict['rightHand']['handPose']
            self.rightPose = np.array((
                (right_pose_dict['e00'], right_pose_dict['e01'], right_pose_dict['e02'], right_pose_dict['e03']),
                (right_pose_dict['e10'], right_pose_dict['e11'], right_pose_dict['e12'], right_pose_dict['e13']),
                (right_pose_dict['e20'], right_pose_dict['e21'], right_pose_dict['e22'], right_pose_dict['e23']),
                (0, 0, 0, 1)
            ))

            left_pose_dict = state_dict['leftHand']['handPose']
            self.leftPose = np.array((
                (left_pose_dict['e00'], left_pose_dict['e01'], left_pose_dict['e02'], left_pose_dict['e03']),
                (left_pose_dict['e10'], left_pose_dict['e11'], left_pose_dict['e12'], left_pose_dict['e13']),
                (left_pose_dict['e20'], left_pose_dict['e21'], left_pose_dict['e22'], left_pose_dict['e23']),
                (0, 0, 0, 1)
            ))

            self.right_hand_command = state_dict['rightHand']['isHandOpened']
            self.left_hand_command = state_dict['leftHand']['isHandOpened']
        else:
            raise NameError('NoHandInformationsInWebSocket')

    def goto_right_arm_joint_solution(self, joint_solution, duration, wait):
        """Move reachy's right arm according to given joints."""
        for joint_pos, motor in zip(joint_solution, self.right_arm.motors):
            motor.goto(joint_pos, duration)

    def goto_left_arm_joint_solution(self, joint_solution, duration, wait):
        """Move reachy's left arm according to given joints."""
        for joint_pos, motor in zip(joint_solution, self.left_arm.motors):
            motor.goto(joint_pos, duration)

    def open_hand(self, arm):
        """Open Reachy's hand."""
        arm.hand.open()

    def close_hand(self, arm):
        """Close Reachy's hand."""
        arm.hand.close()
