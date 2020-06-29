"""tele-operation part module."""
import time
import numpy as np

from threading import Thread

class Tele_op :
    """Class to be use by tele-operation applications

    Args:

        reachy (:py:class:`~reachy.Reachy`): robot which will follow the movements

        left_arm (reachy.parts.LeftArm): left arm part if present or None if absent

        right_arm (reachy.parts.RightArm): right arm part if present or None if absent

    Provides simple method to make reachy follow hands position, rotation and hand opening.
    """

    def __init__(self, left_arm, right_arm):
        """Create the tele-operation instance."""
        self.left_arm = left_arm
        self.right_arm = right_arm
        self.isMoving = False

    def update_state(self, state_dict):
        rightPoseDict = state_dict['rightHand']['handPose']
        self.rightPose = np.array((
            (rightPoseDict['e00'], rightPoseDict['e01'], rightPoseDict['e02'], rightPoseDict['e03']),
            (rightPoseDict['e10'], rightPoseDict['e11'], rightPoseDict['e12'], rightPoseDict['e13']),
            (rightPoseDict['e20'], rightPoseDict['e21'], rightPoseDict['e22'], rightPoseDict['e23']),
            (0, 0, 0, 1)
        ))

        leftPoseDict = state_dict['leftHand']['handPose']
        self.leftPose = np.array((
            (leftPoseDict['e00'], leftPoseDict['e01'], leftPoseDict['e02'], leftPoseDict['e03']),
            (leftPoseDict['e10'], leftPoseDict['e11'], leftPoseDict['e12'], leftPoseDict['e13']),
            (leftPoseDict['e20'], leftPoseDict['e21'], leftPoseDict['e22'], leftPoseDict['e23']),
            (0, 0, 0, 1)
        ))

        self.rightHandCommand = state_dict['rightHand']['isHandOpened']
        self.leftHandCommand = state_dict['leftHand']['isHandOpened']

    def goto_right_arm_joint_solution(self, joint_solution, duration, wait):
        for joint_pos, motor in zip(joint_solution, self.right_arm.motors):
            motor.goto(joint_pos, duration)

    def goto_left_arm_joint_solution(self, joint_solution, duration, wait):
        for joint_pos, motor in zip(joint_solution, self.left_arm.motors):
            motor.goto(joint_pos, duration)

    def start_moving(self, wait=0):
        """Move along with VR poses of both hands.

        Args:

            wait : float
                time in seconds to wait before start to move        
        """
        time.sleep(wait)

        self.move_t = Thread(target=self.follow_movements, daemon=True)
        self.isMoving = True
        self.move_t.start()        

    def stop_moving(self):
        self.isMoving = False;

    def follow_movements(self):
        self.update_state(self.right_arm.io.ws.state_dict)
        isLeftHandOpened = self.leftHandCommand
        isRightHandOpened = self.rightHandCommand
        while self.isMoving :  
            self.update_state(self.right_arm.io.ws.state_dict)           

            isLeftHandOpened = self.moveArm(self.left_arm, self.leftHandCommand, isLeftHandOpened)
            isRightHandOpened = self.moveArm(self.right_arm, self.rightHandCommand, isRightHandOpened)

    def moveArm(self, arm, handCommand, isHandOpened):
        """
        Args:
        
            arm (reachy.parts.arm): the arm to move

            handCommand (bool): true if the hand is supposed to be opened/closed according to the state received by the webSocket, false otherwise

            isHandOpened (bool): the current hand's state 

        The boleans arguments are used to check wether the state has changed or not. It allows us not to call the open()/close() method when 
        the hand is already opened/closed.
        """
        if(handCommand != isHandOpened):
            isHandOpened = handCommand
            if(isHandOpened):
                self.open_t = Thread(target=self.openHand, args=(arm,), daemon=True)
                self.open_t.start()
            else:
                self.close_t = Thread(target=self.closeHand, args=(arm,), daemon=True)
                self.close_t.start()
        
        current_position = [m.present_position for m in arm.motors]
        if(arm.side == self.left_arm.side):
            targetJoint = arm.inverse_kinematics(self.leftPose, q0= current_position)
            self.goto_left_arm_joint_solution(targetJoint, duration=0.1, wait=True)
        else:
            targetJoint = arm.inverse_kinematics(self.rightPose, q0= current_position)
            self.goto_right_arm_joint_solution(targetJoint, duration=0.1, wait=True)

        return isHandOpened

    def openHand(self, arm):
        arm.hand.open();

    def closeHand(self, arm):
        arm.hand.close();
