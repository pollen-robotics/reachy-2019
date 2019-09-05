import time
import numpy as np

from threading import Event, Thread


class TrajectoryRecorder(object):
    def __init__(self, motors, position_field='present_position', freq=50):
        self.motors = motors

        self._data = []
        self._position_field = position_field

        self._recording = Event()
        self._record_t = None

        self.freq = freq

    def start(self, turn_compliant=False):
        self._recording.set()
        self._record_t = Thread(target=self._record_loop)

        if turn_compliant:
            for m in self.motors:
                m.compliant = True

        self._record_t.start()

    def stop(self, turn_stiff=False):
        self._recording.clear()
        if self._record_t is not None and self._record_t.is_alive():
            self._record_t.join()

        if turn_stiff:
            for m in self.motors:
                m.compliant = False

    @property
    def trajectories(self):
        traj = np.array(self._data).T

        return {
            self.motors[i].name: traj
            for i, traj in enumerate(traj)
        }

    def _record_loop(self):
        self._data.clear()

        while self._recording.is_set():
            self._data.append([
                getattr(m, self._position_field)
                for m in self.motors
            ])

            time.sleep(1 / self.freq)
