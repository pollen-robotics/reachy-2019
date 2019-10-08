from operator import attrgetter

from .parts import LeftArm, RightArm, Head


class Reachy(object):
    def __init__(self,
                 left_arm=None,
                 right_arm=None,
                 head=None):

        self._parts = []

        if left_arm is not None:
            if not isinstance(left_arm, LeftArm):
            raise ValueError('"left_arm" must be a LeftArm or None!')
            self._parts.append(left_arm)
            if left_arm.hand is not None:
                self._parts.append(left_arm.hand)
        self.left_arm = left_arm

        if right_arm is not None:
            if not isinstance(right_arm, RightArm):
            raise ValueError('"right_arm" must be a RightArm or None!')
            self._parts.append(right_arm)
            if right_arm.hand is not None:
                self._parts.append(right_arm.hand)
        self.right_arm = right_arm

        if head is not None:
            if not isinstance(head, Head):
            raise ValueError('"head" must be a Head or None!')
            self._parts.append(head)
        self.head = head

        logger.info(
            'Connected to reachy',
            extra={
                'parts': [p.name for p in self.parts]
            }
        )

    @property
    def parts(self):
        return self._parts

    def goto(self,
             goal_positions, duration,
             starting_point='present_position',
             wait=False, interpolation_mode='linear'):

        for i, (motor_name, goal_pos) in enumerate(goal_positions.items()):
            last = (i == len(goal_positions) - 1)

            motor = attrgetter(motor_name)(self)
            motor.goto(goal_pos, duration, starting_point,
                       wait=last, interpolation_mode=interpolation_mode)
