"""Trajectory interpolation utility module.

This module defines various interpolation technique (linear, minimum jerk).
They can be used in all goto functions.
"""
import time
import numpy as np

from threading import Thread, Event
from scipy.interpolate import interp1d


class TrajectoryInterpolation(object):
    """Trajectory interpolation abstraction class.

    Args:
        initial_position (float): starting position (in degrees)
        goal_position (float): end position (in degrees)
        duration (float): duration of the movement (in seconds)

    You can defined your own interpolation technique by respecting this abstraction so they can be used in goto functions.
    """

    def __init__(self, initial_position, goal_position, duration):
        """Create your interpolation object."""
        self.initial_position = initial_position
        self.goal_position = goal_position
        self.duration = duration

        self._t = None
        self._running = Event()

    def interpolate(self, t):
        """Interpolate the position at given time.

        Args:
            t (float): time where to interpolate

        You are responsible for implementing this method in your own interpolation technique.
        Please refer to the implementation of Linear of MinimumJerk for examples.
        """
        raise NotImplementedError

    def start(self, motor, update_freq=100):
        """Start the interpolation trajectory thread.

        Args:
            motor (motor): motor to apply the trajectory to
            update_freq (float): Update sample frequency (in Hz)
        """
        self._t = Thread(target=lambda: self._follow_traj_loop(motor, update_freq))
        self._running.set()
        self._t.start()

    @property
    def is_playing(self):
        """Check if the trajectory is currently playing."""
        return self._t is not None and self._t.is_alive()

    def stop(self, wait=True):
        """Stop the interpolation trajectory."""
        self._running.clear()
        if wait:
            self.wait()

    def wait(self):
        """Block until the end of the trajectory interpolation."""
        if self.is_playing:
            self._t.join()

    def _follow_traj_loop(self, motor, update_freq):
        t0 = time.time()

        while self._running.is_set():
            t = time.time() - t0
            if t > self.duration:
                break

            pos = self.interpolate(t)
            if hasattr(motor, 'goal_position'):
                motor.goal_position = pos
            else:
                motor.target_rot_position = pos

            time.sleep(1 / update_freq)


class Linear(TrajectoryInterpolation):
    """Linear implementation implementation."""

    def interpolate(self, t):
        """Linear interpolation at time t."""
        return self.initial_position + (self.goal_position - self.initial_position) * t / self.duration


class MinimumJerk(TrajectoryInterpolation):
    """Minimum Jerk interpolation implementation.

    Args:
        initial_position (float): starting position (in degrees)
        goal_position (float): end position (in degrees)
        duration (float): duration of the movement (in seconds)
        initial_velocity (float): initial velocity used for interpolation
        final_velocity (float): final velocity used for interpolation
        initial_acceleration (float): initial acceleration used for interpolation
        final_acceleration (float): final acceleration used for interpolation
    """

    def __init__(
        self,
        initial_position, goal_position, duration,
        initial_velocity=0, final_velocity=0, initial_acceleration=0, final_acceleration=0,
    ):
        """Create the minjerk interpolation."""
        TrajectoryInterpolation.__init__(self, initial_position, goal_position, duration)

        a0 = initial_position
        a1 = initial_velocity
        a2 = initial_acceleration / 2

        d1, d2, d3, d4, d5 = [duration ** i for i in range(1, 6)]

        A = np.array((
            (d3, d4, d5),
            (3 * d2, 4 * d3, 5 * d4),
            (6 * d1, 12 * d2, 20 * d3)
        ))
        B = np.array((
            goal_position - a0 - (a1 * d1) - (a2 * d2),
            final_velocity - a1 - (2 * a2 * d1),
            final_acceleration - (2 * a2)
        ))
        X = np.linalg.solve(A, B)

        self._coeffs = [
            a0,
            a1,
            a2,
            X[0],
            X[1],
            X[2]
        ]

    def interpolate(self, t):
        """Minjerk interpolation at time t."""
        return np.sum([
            c * t ** i
            for i, c in enumerate(self._coeffs)
        ], axis=0)


def cubic_smooth(traj, nb_kp, out_points=-1):
    """Trjaectory cubic smooth interpolation.

    Args:
        traj (dict): trajectory to smooth ({motor_name: list of motor pos})
        nb_kp (int): number of keypoints to use for the cubic smoothing
        out_points (int): number of samples in the output trajectory (use -1 to conserve the same number as the input trajectory)
    """
    as_dict = isinstance(traj, dict) or isinstance(traj, np.lib.npyio.NpzFile)

    if as_dict:
        Y = np.array(list(traj.values())).T
    else:
        Y = traj

    X = np.linspace(0, 1, Y.shape[0], endpoint=True)
    fY = interp1d(X, Y.T, kind='linear')

    kx = np.linspace(0, 1, nb_kp)
    KP = fY(kx)

    if out_points == -1:
        out_points = len(X)

    C = interp1d(kx, KP, kind='cubic')
    SY = C(np.linspace(0, 1, out_points, endpoint=True)).T

    if as_dict:
        return {
            m: SY[:, i]
            for i, m in enumerate(traj.keys())
        }
    else:
        return SY


interpolation_modes = {
    'linear': Linear,
    'minjerk': MinimumJerk,
}
