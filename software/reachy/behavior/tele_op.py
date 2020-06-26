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

    def __init__(self, reachy):
        """Create the tele-operation instance."""
        self._reachy = reachy
        self.left_arm = _reachy.left_arm
        self.right_arm = _reachy.right_arm
    
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

        self.isRightHandOpened = state_dict['rightHand']['isHandOpened']
        self.isLeftHandOpened = state_dict['leftHand']['isHandOpened']


    def goto_right_arm_joint_solution(self, joint_solution, duration, wait):
        self._reachy.goto({
            m.name: j
            for j, m in zip(joint_solution, self.right_arm.motors)
        }, duration=duration, wait=wait)
    
    
    def goto_left_arm_joint_solution(self, joint_solution, duration, wait):
        self._reachy.goto({
            m.name: j
            for j, m in zip(joint_solution, self.left_arm.motors)
        }, duration=duration, wait=wait)

        for joint_pos, motor in zip(joint_solution, self.left_arm.motors):
            motor.goto(joint_pos, duration)


    def start_moving(self, wait=0):
        """Move along with VR poses of both hands.

        Args:
            wait (float): time in seconds to wait before start to move        
        """
        # handle wait 
        time.sleep(wait)

        self._play_t = Thread(target=self.follow_movements, daemon=True)
        self._play_t.start()


    def follow_movements(self):
        leftHandOpen = self.isLeftHandOpened
        rightHandOpen = self.isRightHandOpened
        while True :  
            ## Move right hand               
            if(self.isLeftHandOpened != leftHandOpen):
                leftHandOpen = self.isLeftHandOpened
                if(leftHandOpen):
                    self.left_arm.hand.open()
                else:
                    self.left_arm.hand.close()
            
            current_position = [m.present_position for m in self.left_arm.motors]
            targetJoint = self.left_arm.inverse_kinematics(self.leftPose, q0= current_position)
            self.goto_left_arm_joint_solution(targetJoint, duration=0.1, wait=True)
            
            # Move right hand             
            if(self.isRightHandOpened != rightHandOpen):
                rightHandOpen = self.isRightHandOpened
                if(rightHandOpen):
                    self.right_arm.hand.open()
                    
                else:
                    self.right_arm.hand.close()
            
            current_position = [m.present_position for m in self.right_arm.motors]
            targetJoint = self.right_arm.inverse_kinematics(self.rightPose, q0= current_position)
            self.goto_right_arm_joint_solution(targetJoint, duration=0.1, wait=True)
