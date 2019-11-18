import time
import numpy as np

from threading import Thread
from operator import attrgetter


class TrajectoryPlayer(object):
    def __init__(self, reachy, trajectories, freq=100):
        motor_names, trajectories = zip(*trajectories.items())

        self._motors = [attrgetter(name)(reachy) for name in motor_names]
        self._traj = np.array(trajectories).T

        self._play_t = None

        self.freq = freq

    def play(self, wait=False):
        self._play_t = Thread(target=self._play_loop)
        self._play_t.start()

        if wait:
            self.wait_for_end()

    def wait_for_end(self):
        if self._play_t is not None and self._play_t.is_alive():
            self._play_t.join()

    def _play_loop(self):
        for pt in self._traj:
            for i, m in enumerate(self._motors):
                m.goal_position = pt[i]

            time.sleep(1 / self.freq)
