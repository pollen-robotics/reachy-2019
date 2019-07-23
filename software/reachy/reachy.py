from .parts import LeftArm, RightArm


class Reachy(object):
    def __init__(self,
                 left_arm=None,
                 right_arm=None):

        if left_arm is not None and not isinstance(left_arm, LeftArm):
            raise ValueError('"left_arm" must be a LeftArm or None!')
        self.left_arm = left_arm

        if right_arm is not None and not isinstance(right_arm, RightArm):
            raise ValueError('"right_arm" must be a RightArm or None!')
        self.right_arm = right_arm
