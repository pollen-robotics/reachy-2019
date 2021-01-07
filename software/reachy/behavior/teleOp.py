"""tele-operation part module."""
import time
import numpy as np
from scipy.spatial.transform import Rotation
from reachy_kdl import inverse_kinematics, forward_kinematics
from threading import Thread
from pyquaternion import Quaternion
from collections import deque


class TeleOp(object):
    """Class to be use by tele-operation applications.
    Args:
        reachy (:py:class:`~reachy.Reachy`): robot which will follow the movements
        left_arm (reachy.parts.LeftArm): left arm part if present or None if absent
        right_arm (reachy.parts.RightArm): right arm part if present or None if absent
    Provides easy-to-use method to make reachy follow hands position, rotation and hand opening.
    """

    def __init__(self, left_arm, right_arm, head):
        """Create the tele-operation instance."""
        self.left_arm = left_arm
        self.right_arm = right_arm
        self.head = head
        self.is_moving = False
        
        self.right_opening = False
        self.left_opening = False

        self.last_updates = deque([], 100)
        self.last_dts = deque([], 100)

        self.pierre_log = []

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

            ## Using ROS package
            res, sol_right = inverse_kinematics(
                label='right_arm',
                q0=[0, 0, 0, -90, 0, 0, 0],
                target_pose=self.rightPose,
            )
            self.goto_right_arm_joint_solution(sol_right, duration=0.1)
            res, sol_left = inverse_kinematics(
                label='left_arm',
                q0=[0, 0, 0, -90, 0, 0, 0],
                target_pose=self.leftPose,
            )
            self.goto_left_arm_joint_solution(sol_left, duration=0.1)

            ## USE TO CONTROL GRIPPER WITH FIXED OPEN/CLOSE VALUES USING B BUTTON
            #is_left_hand_opened = self.move_arm(self.left_arm, self.left_hand_command, is_left_hand_opened)
            #is_right_hand_opened = self.move_arm(self.right_arm, self.right_hand_command, is_right_hand_opened)
            ##
            
            ## USE TO CONTROL GRIPPER WITH ADAPTATIVE VALUES USING TRIGGER
            self.close_hand_2(self.left_arm, self.trigger_left)
            self.close_hand_2(self.right_arm, self.trigger_right)
            ##

            self.head_orient(self.head_quaternion, duration=0.01, wait=False)

            time.sleep(0.01)
            

            # self.pierre_log.append(([m.present_position for m in self.right_arm.motors[:7]], self.rightPose, toc - tic))



            #is_left_hand_opened = self.move_arm(self.left_arm, self.left_hand_command, is_left_hand_opened)
            #is_right_hand_opened = self.move_arm(self.right_arm, self.right_hand_command, is_right_hand_opened)

            # self.right_hand.append(self.rightPose)
            # self.left_hand.append(self.leftPose)

            ## Résolution plus proche voisin
            # self.rightPose_flat = np.reshape(self.rightPose, (1,4,4))
            # joint_solution = self.scaler.inverse_transform(self.IK_model.predict(self.rightPose_flat))[0]
            # self.goto_right_arm_joint_solution(joint_solution, duration=0.1, wait=True)

            ## With Euler angles
            # forward2euler = Rotation.from_matrix(self.rightPose[:3, :3]).as_euler('xyz')
            # rot_and_pos = np.array([forward2euler, self.rightPose[:3, 3].T])
            # rot_and_pos = np.reshape(rot_and_pos, (1, 2, 3))
            # joint_solution = self.scaler.inverse_transform(self.IK_model.predict(rot_and_pos))[0]
            # if (self.ind < 173288):
            #     joint_solution = self.data[self.ind]
            #     self.goto_right_arm_joint_solution(joint_solution, duration=0.1, wait=True)
            #     self.ind += 1
            #     print(self.ind)
            #     self.right_arm.ind = self.ind
            #     print(self.right_arm.ind)

            ## Résolution IK à la suite
            # is_left_hand_opened = self.move_arm(self.left_arm, self.left_hand_command, is_left_hand_opened)
            # is_right_hand_opened = self.move_arm(self.right_arm, self.right_hand_command, is_right_hand_opened)

            ## Résolution IK simultanée
            # current_position_left = [m.present_position for m in self.left_arm.motors]
            # self.move_left = Thread(target=self.IK_calculation, args=(self.left_arm,current_position_left,))
            # self.move_left.start()
            # current_position_right = [m.present_position for m in self.left_arm.motors]
            # self.move_right =  Thread(target=self.IK_calculation, args=(self.right_arm,current_position_right,))
            # self.move_right.start()

            # for calc in [self.move_left, self.move_right]:
            #     calc.join()

            # self.look_at_new_point(self.head_rotation, duration=0.1, wait=False)
           
            #pierre
            # print("Execution time : ", time.time() - tic)

    def move_arm(self, arm, hand_command, is_hand_opened):
        """Move one arm according to the pose given by update_state.
        Args:
            arm (reachy.parts.arm): the arm to move
            hand_command (bool): true if the hand is supposed to be opened/closed according to the state received by the webSocket, false otherwise
            is_hand_opened (bool): the current hand's state
        The boleans arguments are used to check wether the state has changed or not. It allows us not to call the open()/close() method when
        the hand is already opened/closed.
        """
        if(hand_command != is_hand_opened and not self.right_opening):
            is_hand_opened = hand_command
            
            if(is_hand_opened):
                self.open_hand(arm)
            else:
                self.close_hand(arm)

        # current_position = [m.present_position for m in arm.motors]

        # if(arm.side == self.left_arm.side):
        #     tic_l = time.time()
        #     targetJoint = arm.inverse_kinematics(self.leftPose, q0=current_position)
        #     tac_l = time.time()
        #     duration_l = tac_l -tic_l
        #     #print(duration_l)
        #     #print('left : ')
        #     #print(targetJoint)
        #     self.IKleft.append(duration_l)
        #     self.goto_left_arm_joint_solution(targetJoint, duration=0.1, wait=True)
        # else:
        #     tic_r = time.time()
        #     targetJoint = arm.inverse_kinematics(self.rightPose, q0=current_position)
        #     tac_r = time.time()
        #     duration_r = tac_r - tic_r
        #     #print(duration_r)
        #     #print('right : ')
        #     #print(targetJoint)
        #     self.IKright.append(duration_r)
        #     self.goto_right_arm_joint_solution(targetJoint, duration=0.1, wait=True)

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

            # self.head_rotation = state_dict['head_rot']
            self.head_quaternion = state_dict['head_quat']

            self.trigger_left = state_dict['leftHand']['trigger']
            self.trigger_right = state_dict['rightHand']['trigger']
            # print(self.head_rotation)

        else:
            raise NameError('NoHandInformationsInWebSocket')

    def goto_right_arm_joint_solution(self, joint_solution, duration):
        """Move reachy's right arm according to given joints."""
        for joint_pos, motor in zip(joint_solution, self.right_arm.motors):
            # motor.goto(joint_pos, duration)
            motor.goal_position = joint_pos

    def goto_left_arm_joint_solution(self, joint_solution, duration):
        """Move reachy's left arm according to given joints."""
        for joint_pos, motor in zip(joint_solution, self.left_arm.motors):
            # motor.goto(joint_pos, duration)
            motor.goal_position = joint_pos

    def look_at_new_point(self, head_rotation, duration, wait):
        y = np.tan(np.deg2rad(head_rotation['z']))
        z = -np.tan(np.deg2rad(head_rotation['y']))
        self.head.look_at(1, y, z, duration=duration, wait=False)

    def head_orient(self, head_quaternion, duration, wait):
        q = Quaternion(head_quaternion['w'], head_quaternion['y'], head_quaternion['x'], -head_quaternion['z'])
        #print(head_quaternion)
        #print(q)
        try:
            thetas = self.head.neck.model.get_angles_from_quaternion(q.w,q.x,q.y,q.z)
            self.head.neck.disk_top.target_rot_position = thetas[0]
            self.head.neck.disk_middle.target_rot_position = thetas[1]
            self.head.neck.disk_bottom.target_rot_position = thetas[2]
        except ValueError:
            print("math domain error")

    def open_hand(self, arm, pos=-30):
        """Open Reachy's hand."""
        if(arm.side == self.left_arm.side):
            pos = -pos
        arm.hand.gripper.goal_position = pos

    def close_hand(self, arm, pos=6):
        """Close Reachy's hand."""
        if(arm.side == self.left_arm.side):
            pos = -8
        arm.hand.gripper.goal_position = pos        

    def close_hand_2(self, arm, trigger):
        pos = trigger*20 - 40*(1-trigger)
        if(arm.side == self.left_arm.side):
            pos = -pos 
        arm.hand.gripper.goal_position = pos
    
    def IK_calculation(self, arm, current_position):
        if(arm.side == self.left_arm.side):
            tic = time.time()
            targetJoint = arm.inverse_kinematics(self.leftPose, q0=current_position)
            print(time.time() - tic)
            self.goto_left_arm_joint_solution(targetJoint, duration=0.1, wait=True)
        else:
            tic = time.time()
            targetJoint = arm.inverse_kinematics(self.rightPose, q0=current_position)
            print(time.time() - tic)
            self.goto_right_arm_joint_solution(targetJoint, duration=0.1, wait=True)


def position_dist(P, Q):
    """Compute euclidian distance between two 3D position."""
    return np.linalg.norm(P - Q)


def rotation_dist(P, Q):
    """Compute rotation distance between two 3D rotation."""
    R = np.matmul(P, Q.T)

    A = (np.trace(R) - 1) / 2
    A = np.clip(A, -1, 1)

    theta = np.arccos(A)
    return theta