"""Trajectory recording utility module."""

import time
import numpy as np

from threading import Event, Thread


class TrajectoryRecorder(object):
    """Trajectory Recorder utility class.

    Args:
        motors (list): list of motors to record (eg. [reachy.right_arm.elbow_pitch, reachy.right_arm.shoulder_pitch])
        position_field (str): register to record as trajectories (default use the 'present_position', 'goal_position' can also be useful in some specific case)
        freq (float): record sample frequency (in Hz)

    .. note:: A same recorder can be used to record multiple trajectories.

    Facilitates the recording of a full trajectory on multiple motors.
    """

    def __init__(self, motors, position_field='present_position', freq=100):
        """Create the recorder."""
        self.motors = motors

        self._data = []
        self._position_field = position_field

        self._recording = Event()
        self._record_t = None

        self.freq = freq

    def start(self, turn_compliant=False):
        """Start the record.

        Args:
            turn_compliant (bool): whether or not to turn the motor compliant before starting the record.
        """
        self._recording.set()
        self._record_t = Thread(target=self._record_loop)

        if turn_compliant:
            for m in self.motors:
                m.compliant = True

        self._record_t.start()

    def stop(self, turn_stiff=False):
        """Stop the record.

        Args:
            turn_stiff (bool): whether or not to turn the motor stiff at the end of the record.
        """
        self._recording.clear()
        if self._record_t is not None and self._record_t.is_alive():
            self._record_t.join()

        if turn_stiff:
            for m in self.motors:
                m.compliant = False

    @property
    def trajectories(self):
        """Retrieve a copy of the recorded trajectories."""
        traj = np.array(self._data).T

        return {
             self.motors[i].name: t
             for i, t in enumerate(traj)
         }

    def _record_loop(self):
        self._data.clear()

        while self._recording.is_set():
            self._data.append([
                getattr(m, self._position_field)
                for m in self.motors
            ])

            time.sleep(1 / self.freq)
