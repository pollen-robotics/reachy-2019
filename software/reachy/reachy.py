from operator import attrgetter

from .parts import LeftArm, RightArm, Head


class Reachy(object):
    def __init__(self,
                 left_arm=None,
                 right_arm=None,
                 head=None):

        if left_arm is not None and not isinstance(left_arm, LeftArm):
            raise ValueError('"left_arm" must be a LeftArm or None!')
        self.left_arm = left_arm

        if right_arm is not None and not isinstance(right_arm, RightArm):
            raise ValueError('"right_arm" must be a RightArm or None!')
        self.right_arm = right_arm

        if head is not None and not isinstance(head, Head):
            raise ValueError('"head" must be a Head or None!')
        self.head = head

    def goto(self,
             goal_positions, duration,
             starting_point='present_position',
             wait=False, interpolation_mode='linear'):

        for i, (motor_name, goal_pos) in enumerate(goal_positions.items()):
            last = (i == len(goal_positions) - 1)

            motor = attrgetter(motor_name)(self)
            motor.goto(goal_pos, duration, starting_point,
                       wait=last, interpolation_mode=interpolation_mode)
