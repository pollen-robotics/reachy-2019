"""Trajectory player module."""

import time
import numpy as np

from threading import Thread
from operator import attrgetter


class TrajectoryPlayer(object):
    """Trajectory player abstraction.

    Args:
        reachy (:py:class:`~reachy.Reachy`): robot which will play the trajectory
        trajectories (dict): Trajectory to play (such as {motor_name: list of positions})
        freq (float): Replay sample frequency (in Hz)

    Provides high-level features to:
        * play a pre-defined trajectory
        * wait for the end of the replay
        * add fade in to smooth begining of motion
    """

    def __init__(self, reachy, trajectories, freq=100):
        """Create the Trajectory Player."""
        motor_names, trajectories = zip(*trajectories.items())

        self._reachy = reachy
        self._motors = [attrgetter(name)(reachy) for name in motor_names]
        self._traj = np.array(trajectories).T

        self._play_t = None

        self.freq = freq

    def play(self, wait=False, fade_in_duration=0):
        """Play a given trajectory.

        Args:
            wait (bool): whether or not to wait for the end of the trajectory replay
            fade_in_duration (float): time in seconds to reach the starting position (can be used to smooth the begining of the motion)

        .. warning:: you can call play multiple times to replay the trajectory. You are responisble for handling the concurrency issue if you try to run multiple trajectories on the same motors.
        """
        if fade_in_duration > 0:
            self._reachy.goto(
                goal_positions=dict(zip([m.name for m in self._motors], self._traj[0, :])),
                duration=fade_in_duration,
                starting_point='goal_position',
                wait=True,
                interpolation_mode='minjerk',
            )

        self._play_t = Thread(target=self._play_loop)
        self._play_t.start()

        if wait:
            self.wait_for_end()

    def wait_for_end(self):
        """Block until the end of a trajectory replay."""
        if self._play_t is not None and self._play_t.is_alive():
            self._play_t.join()

    def _play_loop(self):
        for pt in self._traj:
            for i, m in enumerate(self._motors):
                m.goal_position = pt[i]

            time.sleep(1 / self.freq)
