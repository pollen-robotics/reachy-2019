import time
import numpy as np

from threading import Thread, Event
from scipy.interpolate import interp1d


class TrajectoryInterpolation(object):
    def __init__(self, initial_position, goal_position, duration):
        self.inital_position = initial_position
        self.goal_position = goal_position
        self.duration = duration

        self._t = None
        self._running = Event()

    def interpolate(self, t):
        raise NotImplementedError

    def start(self, motor, update_freq=50):
        self._t = Thread(target=lambda: self._follow_traj_loop(motor, update_freq))
        self._running.set()
        self._t.start()

    def stop(self):
        self._running.clear()

    def wait(self):
        if self._t is not None and self._t.is_alive():
            self._t.join()

    def _follow_traj_loop(self, motor, update_freq):
        t0 = time.time()

        while self._running.is_set():
            t = time.time() - t0
            if t > self.duration:
                break

            pos = self.interpolate(t)
            motor.goal_position = pos

            time.sleep(1 / update_freq)


class Linear(TrajectoryInterpolation):
    def interpolate(self, t):
        return self.inital_position + (self.goal_position - self.inital_position) * t / self.duration


class MinimumJerk(TrajectoryInterpolation):
    def __init__(
        self,
        initial_position, goal_position, duration,
        initial_velocity=0, final_velocity=0, initial_acceleration=0, final_acceleration=0,
    ):
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
        return np.sum([
            c * t ** i
            for i, c in enumerate(self._coeffs)
        ], axis=0)


def cubic_smooth(Y, nb_points):
    X = np.linspace(0, 1, Y.shape[0], endpoint=True)
    fY = interp1d(X, Y.T, kind='linear')

    kx = np.linspace(0, 1, nb_points)
    KP = fY(kx)

    C = interp1d(kx, KP, kind='cubic')
    return C(X).T
