"""Reachy main module entry point."""

import time
import logging
import numpy as np

from operator import attrgetter

from .parts import LeftArm, RightArm, Head


logger = logging.getLogger(__name__)


class Reachy(object):
    """Class representing the connection with the hardware robot.

    Args:
        left_arm (reachy.parts.LeftArm): left arm part if present or None if absent
        right_arm (reachy.parts.RightArm): right arm part if present or None if absent
        head (reachy.parts.Head): hrad part if present or None if absent

    Connect and synchronize with the hardware robot.

    It can be used to monitor real time robot state and to send commands.
    Mainly a container to hold the different parts of Reachy together.
    """

    def __init__(self,
                 left_arm=None,
                 right_arm=None,
                 head=None):
        """
        Connect and synchronize with the hardware robot.

        Args:
            left_arm (reachy.parts.LeftArm): left arm part if present or None if absent
            right_arm (reachy.parts.RightArm): right arm part if present or None if absent
            head (reachy.parts.Head): hrad part if present or None if absent
        """
        self._parts = []

        if left_arm is not None:
            if not isinstance(left_arm, LeftArm):
                raise ValueError('"left_arm" must be a LeftArm or None!')
            self._parts.append(left_arm)
        self.left_arm = left_arm

        if right_arm is not None:
            if not isinstance(right_arm, RightArm):
                raise ValueError('"right_arm" must be a RightArm or None!')
            self._parts.append(right_arm)
        self.right_arm = right_arm

        if head is not None:
            if not isinstance(head, Head):
                raise ValueError('"head" must be a Head or None!')
            self._parts.append(head)
        self.head = head

        logger.info(
            'Connected to reachy',
            extra={
                'parts': [p.name for p in self.parts],
            }
        )

    def __repr__(self):
        """Reachy representation."""
        return f'<Reachy: {self.parts}>'

    def close(self):
        """Close all communication with each attached parts."""
        logger.info(
            'Closing connection with reachy',
            extra={
                'parts': [p.name for p in self.parts],
            }
        )
        for p in self.parts:
            p.teardown()

    @property
    def parts(self):
        """List of all attached parts."""
        return self._parts

    @property
    def motors(self):
        """List of all motors in the attached parts."""
        return sum([p.motors for p in self.parts], [])

    def goto(self,
             goal_positions, duration,
             starting_point='present_position',
             wait=False, interpolation_mode='linear'):
        """
        Goto specified goal positions.

        Args:
            goal_positions (dict): desired target position (in the form {'full_motor_name': target_position})
            duration (float): move duration (in sec.)
            starting_point (str): register to use to retrieve the starting point (e.g. 'present_postion' or 'goal_position')
            wait (bool): whether or not to wait for the end motion before returning
            interpolation_mode (str): interpolation used for computing the trajectory (e.g. 'linear' or 'minjerk')

        Returns:
            list: list of reachy.trajectory.TrajectoryPlayer (one for each controlled motor)

        """
        trajs = []

        for i, (motor_name, goal_pos) in enumerate(goal_positions.items()):
            last = wait and (i == len(goal_positions) - 1)

            motor = attrgetter(motor_name)(self)
            trajs.append(motor.goto(goal_pos, duration, starting_point,
                         wait=last, interpolation_mode=interpolation_mode))

        return trajs

    def need_cooldown(self, temperature_limit=50):
        """
        Check if Reachy needs to cool down.

        Args:
            temperature_limit (int): Temperature limit (in °C) for each motor.

        Returns:
            bool: Whether or not you should let the robot cool down

        """
        motor_temperature = np.array([
            m.temperature for m in self.motors
        ])
        logger.info(
            'Checking Reachy motors temperature',
            extra={
                'temperatures': {
                    m.name: m.temperature for m in self.motors
                }
            }
        )
        return np.any(motor_temperature > temperature_limit)

    def wait_for_cooldown(self, rest_position, goto_rest_duration=5, lower_temperature=45):
        """
        Wait for the robot to lower its temperature.

        The robot will first go to the specified rest position and then, it will turn all motors compliant.
        Finally, it will wait until the temperature of each motor goes below the lower_temperature parameters.

        Args:
            rest_position (dict): the desired rest position for the robot
            goto_rest_duration (float): time in seconds to reach the rest position
            lower_temeprature (int): lower temperature threshold (in °C) to be reached by all motors before the end of cool down

        .. note:: The robot will stay compliant at the end of the function call.
                  It is up to you, to put it back in the desired position.

        """
        self.goto(
            goal_positions=rest_position,
            duration=goto_rest_duration,
            wait=True,
        )

        for m in self.motors:
            m.compliant = True

        while True:
            motor_temperature = np.array([
                m.temperature for m in self.motors
            ])

            logger.warning(
                'Motors cooling down...',
                extra={
                    'temperatures': {
                        m.name: m.temperature for m in self.motors
                    }
                },
            )

            if np.all(motor_temperature < lower_temperature):
                break

            time.sleep(30)
