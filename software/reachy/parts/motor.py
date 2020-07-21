"""Motor abstraction module.

Define:
    * DynamixelMotor
    * OrbitaActuator
"""

import time
import logging
import numpy as np

from threading import Timer
from orbita import Actuator as OrbitaModel

from ..trajectory.interpolation import interpolation_modes

logger = logging.getLogger(__name__)


class DynamixelMotor(object):
    """DynamixelMotor abstraction class.

    Args:
        root_part (str): name of the part where the motor is attached to (eg 'right_arm.hand')
        name (str): name of the motor (eg. 'shoulder_pitch')
        luos_motor (:py:class:`pyluos.modules.DxlMotor`): pyluos motor
        config (dict): extra motor config (must include 'offset' and 'orientation' fields)

    Wrap the pyluos motor object to simplify and make the API homogeneous.
    """

    def __init__(self, root_part, name, luos_motor, config):
        """Create a DynamixelMotor given its pyluos equivalent."""
        self._root_part = root_part
        self._name = name

        self._motor = luos_motor

        self._offset = config['offset']
        self._direct = config['orientation'] == 'direct'

        self._timer = None
        self._use_static_fix = False

    def __repr__(self):
        """Motor representation."""
        mode = 'compliant' if self.compliant else 'stiff'
        return f'<DxlMotor "{self.name}" pos="{self.present_position}" mode="{mode}">'

    @property
    def name(self):
        """Fullname of the motor (eg. right_arm.hand.gripper)."""
        return f'{self._root_part.name}.{self._name}'

    # Position
    @property
    def present_position(self):
        """Present position (in degrees) of the motor."""
        return self._as_local_pos(self._motor.rot_position)

    @property
    def goal_position(self):
        """Get current goal position (in degrees) of the motor."""
        return self._as_local_pos(self._motor.target_rot_position)

    @goal_position.setter
    def goal_position(self, value):
        if not self.compliant:
            self._motor.target_rot_position = self._to_motor_pos(value)

            if self._use_static_fix:
                self._schedule_static_error_fix(delay=1)

    @property
    def offset(self):
        """Get motor real zero (in degrees)."""
        return self._offset

    def is_direct(self):
        """Check whether the motor is direct or not."""
        return self._direct

    def _as_local_pos(self, pos):
        return (pos if self.is_direct() else -pos) - self.offset

    def _to_motor_pos(self, pos):
        return (pos + self.offset) * (1 if self.is_direct() else -1)

    # Speed
    @property
    def moving_speed(self):
        """Get the maximum speed (in degree per second) of the motor."""
        return self._motor.target_rot_speed

    @moving_speed.setter
    def moving_speed(self, value):
        self._motor.target_rot_speed = value

    # Compliancy
    @property
    def compliant(self):
        """Check whether or not the motor is compliant."""
        return self._motor.compliant

    @compliant.setter
    def compliant(self, value):
        self._motor.compliant = value

    @property
    def torque_limit(self):
        """Check the maximum torque allowed (in %) of the motor."""
        return self._motor.power_ratio_limit

    @torque_limit.setter
    def torque_limit(self, value):
        self._motor.power_ratio_limit = value

    # Temperature
    @property
    def temperature(self):
        """Check the current motor temp. (in °C)."""
        return self._motor.temperature

    def goto(self,
             goal_position, duration,
             starting_point='present_position',
             wait=False, interpolation_mode='linear'):
        """Set trajectory goal for the motor.

        Args:
            goal_position (float): target position (in degrees)
            duration (float): duration of the movement (in seconds)
            starting_point (str): register used to determine the starting point (eg. 'goal_position' can also be used in some specific case)
            wait (bool): whether or not to wait for the end of the motion
            interpolation_mode (str): interpolation technique used for computing the trajectory ('linear', 'minjerk')

        Returns:
            reachy.trajectory.TrajectoryPlayer: trajectory player that can be used to monitor the trajectory, stop it, etc
        """
        if interpolation_mode not in interpolation_modes.keys():
            available = tuple(interpolation_modes.keys())
            raise ValueError(f'interpolation_mode should be one of {available}')

        traj_player = interpolation_modes[interpolation_mode](getattr(self, starting_point), goal_position, duration)
        traj_player.start(self)

        if wait:
            traj_player.wait()

        return traj_player

    def use_static_error_fix(self, activate):
        """Trigger the static error fix.

        Args:
            activate (bool): whether to activate/deactivate the static error issue fix

        If activated, the static error fix will check the reach position a fixed delay after the send of a new goal position.
        The static error may result in the motor's load increasing, and yet not managing to move.
        To prevent this behavior we automatically adjust the target goal position to reduce this error.
        """
        self._use_static_fix = activate

    # Patch dynamixel controller issue when the motor forces
    # while not managing to reach the goal position
    def _schedule_static_error_fix(self, delay):
        if self._timer is not None:
            self._timer.cancel()
        self._timer = Timer(delay, self._fix_static_error)
        self._timer.start()

    def _fix_static_error(self, threshold=2):
        error = (self.present_position - self.goal_position)

        if abs(error) > threshold:
            pos = self.goal_position + error / 2
            logger.info('Fix static error controller', extra={
                'goal_position': self.goal_position,
                'present_position': self.present_position,
                'fixed_goal_position': pos,
            })

            self._motor.target_rot_position = self._to_motor_pos(pos)
            self._timer = None


class OrbitaActuator(object):
    """Orbita Actuator abstraction.

    Args:
        root_part (str): name of the part where the motor is attached to (eg 'head')
        name (str): name of the actuator (eg. 'neck')
        disks_motor (list of :py:class:`pyluos.motor_controller`): list of the three disks controllers
        Pc_z (float, float, float): 3D coordinates of the center of the platform (in mm)
        Cp_z (float, float, float): center of the disks rotation circle (in mm)
        R (float): radius of the arms rotation circle around the platform (in mm)
        R0 (:py:class:`~numpy.ndarray`): rotation matrix for the initial rotation
        hardware_zero (float, float, float): absolute hardware zero position of the orbita disks

    Wrap the three disk and the computation model of Orbita to expose higher-level functionalities such as:

    * quaternion control
    * compliancy mode
    * goto
    """

    def __init__(
        self, root_part, name, disks_motor,
        Pc_z, Cp_z, R, R0, hardware_zero
    ):
        """Create a OrbitaActuator given its three disks controllers."""
        self.disk_bottom, self.disk_middle, self.disk_top = disks_motor

        self.model = OrbitaModel(Pc_z=Pc_z, Cp_z=Cp_z, R=R, R0=R0)
        self._hardware_zero = hardware_zero

        self._compliancy = False
        self.setup()

    def __repr__(self):
        """Orbita representation."""
        return (f'<Orbita '
                f'"top disk": {self.disk_top.rot_position} '
                f'"middle disk": {self.disk_middle.rot_position} '
                f'"bottom disk": {self.disk_bottom.rot_position}>')

    @property
    def disks(self):
        """Get three disks [top, middle, bottom]."""
        return [self.disk_top, self.disk_middle, self.disk_bottom]

    @property
    def compliant(self):
        """Check the disks compliancy."""
        return self._compliancy

    @compliant.setter
    def compliant(self, compliancy):
        self._compliancy = compliancy

        for d in self.disks:
            d.compliant = compliancy

    def goto(self,
             thetas, duration, wait,
             interpolation_mode='linear',
             ):
        """Set trajectory goal for three disks.

        Args:
            thetas (float, float, float): target position (in degrees) for each disks (top, middle, bottom)
            duration (float): duration of the movement (in seconds)
            wait (bool): whether or not to wait for the end of the motion
            interpolation_mode (str): interpolation technique used for computing the trajectory ('linear', 'minjerk')

        Returns:
            reachy.trajectory.TrajectoryPlayer: trajectory player that can be used to monitor the trajectory, stop it, etc
        """
        if len(thetas) != len(self.disks):
            raise ValueError(f'Invalid thetas {thetas} (length should be {len(self.disks)}')

        if interpolation_mode not in interpolation_modes.keys():
            available = tuple(interpolation_modes.keys())
            raise ValueError(f'interpolation_mode should be one of {available}')
        Traj = interpolation_modes[interpolation_mode]

        trajs = [
            Traj(
                initial_position=disk.target_rot_position,
                goal_position=angle,
                duration=duration,
            )
            for disk, angle in zip(self.disks, thetas)
        ]

        for disk, traj in zip(self.disks, trajs):
            traj.start(disk)

        if wait:
            for traj in trajs:
                traj.wait()

        return trajs

    def point_at(self, vector, angle, duration, wait):
        """Make orbita point at the given vector.

        Args:
            vector (float, float, float): 3D vector indicating the poiting direction (in m)
            angle (float):  rotation around the vector (in degrees)
            duration (float): move duration (in seconds)
            wait (bool): whether or not to wait for the end of the motion

        Returns:
            reachy.trajectory.TrajectoryPlayer: trajectory player that can be used to monitor the trajectory, stop it, etc
        """
        thetas = self.model.get_angles_from_vector(vector, angle)
        # We used a reversed encoder so we need to inverse the angles
        return self.goto(thetas, duration=duration, wait=wait, interpolation_mode='minjerk')

    def orient(self, quat, duration, wait):
        """Orient orbita given a quaternion.

        Args:
            quat (pyquaternion.Quaterion): quaternion goal orientation
            duration (float): move duration (in seconds)
            wait (bool): whether or not to wait for the end of the motion

        Returns:
            reachy.trajectory.TrajectoryPlayer: trajectory player that can be used to monitor the trajectory, stop it, etc
        """
        thetas = self.model.get_angles_from_quaternion(quat.w, quat.x, quat.y, quat.z)
        # We used a reversed encoder so we need to inverse the angles
        return self.goto(thetas, duration=duration, wait=wait, interpolation_mode='minjerk')

    def setup(self):
        """Configure each of the three disks.

        .. note:: automatically called at instantiation.
        """
        for disk in self.disks:
            disk.setup()

        def _find_zero(disk, z):
            A = 360 / (52 / 24)
            p = disk.rot_position

            zeros = [z, -(A - z), A + z]
            distances = [abs(p - z) for z in zeros]
            best = np.argmin(distances)

            return zeros[best]

        time.sleep(0.25)

        for d, z in zip(self.disks, self._hardware_zero):
            d.offset = _find_zero(d, z) + 60
