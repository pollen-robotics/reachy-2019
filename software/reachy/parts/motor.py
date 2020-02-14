"""Motor abstraction module.

Define:
    * DynamixelMotor
    * OrbitaActuator
"""

import time
import numpy as np

from collections import deque
from pyquaternion import Quaternion
from orbita import Actuator as OrbitaModel

from ..trajectory.interpolation import interpolation_modes


class DynamixelMotor(object):
    """DynamixelMotor abstraction class.

    Wrap the pyluos motor object to simplify and make the API homogeneous.
    """

    def __init__(self, root_part, name, luos_motor, config):
        """Create a DynamixelMotor given its pyluos equivalent.

        Args:
            root_part (str): name of the part where the motor is attached to (eg 'right_arm.hand')
            name (str): name of the motor (eg. 'shoulder_pitch')
            luos_motor (pyluos.Motor): pyluos motor
            config (dict): extra motor config (must include 'offset' and 'orientation' fields)
        """
        self._root_part = root_part
        self._name = name

        self._motor = luos_motor

        self._offset = config['offset']
        self._direct = config['orientation'] == 'direct'

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
        self._motor.target_rot_position = self._to_motor_pos(value)

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
        """Get the current moving speed (in degree per second) of the motor."""
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
            TrajectoryPlayer: trajectory player that can be used to monitor the trajectory, stop it, etc
        """
        if interpolation_mode not in interpolation_modes.keys():
            available = tuple(interpolation_modes.keys())
            raise ValueError(f'interpolation_mode should be one of {available}')

        traj_player = interpolation_modes[interpolation_mode](getattr(self, starting_point), goal_position, duration)
        traj_player.start(self)

        if wait:
            traj_player.wait()

        return traj_player


class OrbitaActuator(object):
    """Orbita Actuator abstraction.

    Wrap the three disk and the computation model of Orbita to expose higher-level functionalities such as:
    * quaternion control
    * compliancy mode
    * goto
    """

    def __init__(
        self, root_part, name, luos_disks_motor,
        Pc_z, Cp_z, R, R0,
        pid, reduction, wheel_size, encoder_res,
    ):
        """Create a OrbitaActuator given its three disks controllers.

        Args:
            root_part (str): name of the part where the motor is attached to (eg 'head')
            name (str): name of the acutator (eg. 'neck')
            luos_disks_motor (list of pyluos.motor_controller): list of the three disks controllers
            Pc_z (float, float, float): TODO
            Cp_z (float, float, float): TODO
            R (float): TODO
            R0 (matrix): rotation matrix for the initial rotation
            pid (float, float, float): coefficient for the pid position controller
            reduction (float): reduction factor
            wheel_size (float): size of the wheel (in mm)
            encoder_res (int): encoder resolution
        """
        self.disk_bottom, self.disk_middle, self.disk_top = luos_disks_motor
        self.model = OrbitaModel(Pc_z=Pc_z, Cp_z=Cp_z, R=R, R0=R0)

        self._compliancy = False

        self.setup(
            pid=pid,
            reduction=reduction,
            wheel_size=wheel_size,
            encoder_res=encoder_res,
        )

    def __repr__(self):
        """Orbita representation."""
        return (f'<Orbita '
                f'"top disk": {self.disk_top.present_position} '
                f'"middle disk": {self.disk_middle.present_position} '
                f'"bottom disk": {self.disk_bottom.present_position}>')

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
            TrajectoryPlayer: trajectory player that can be used to monitor the trajectory, stop it, etc
        """
        if len(thetas) != len(self.disks):
            raise ValueError(f'Invalid thetas {thetas} (length should be {len(self.disks)}')

        if interpolation_mode not in interpolation_modes.keys():
            available = tuple(interpolation_modes.keys())
            raise ValueError(f'interpolation_mode should be one of {available}')
        Traj = interpolation_modes[interpolation_mode]

        trajs = [
            Traj(
                initial_position=disk.rot_position,
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
            TrajectoryPlayer: trajectory player that can be used to monitor the trajectory, stop it, etc
        """
        thetas = self.model.get_angles_from_vector(vector, angle)
        # We used a reversed encoder so we need to inverse the angles
        thetas = [-q for q in thetas]
        return self.goto(thetas, duration=duration, wait=wait, interpolation_mode='minjerk')

    def orient(self, quat, duration, wait):
        """Orient orbita given a quaternion.

        Args:
            quat (pyquaternion.Quaterion): quaternion goal orientation
            duration (float): move duration (in seconds)
            wait (bool): whether or not to wait for the end of the motion

        Returns:
            TrajectoryPlayer: trajectory player that can be used to monitor the trajectory, stop it, etc
        """
        thetas = self.model.get_angles_from_quaternion(quat.w, quat.x, quat.y, quat.z)
        # We used a reversed encoder so we need to inverse the angles
        thetas = [-q for q in thetas]
        self.goto(thetas, duration=duration, wait=wait, interpolation_mode='minjerk')

    def setup(self, pid, reduction, wheel_size, encoder_res):
        """Configure each of the three disks.

        Args:
            pid (float, float, float): coefficient for the pid position controller
            reduction (float): reduction factor
            wheel_size (float): size of the wheel (in mm)
            encoder_res (int): encoder resolution

        .. note:: automatically called at instantiation.
        """
        for disk in self.disks:
            disk.limit_current = 0.4
            disk.encoder_res = encoder_res
            disk.reduction = reduction
            disk.wheel_size = wheel_size
            disk.positionPid = pid
            disk.rot_position_mode = True
            disk.rot_speed_mode = False
            disk.rot_position = True
            disk.rot_speed = False
            disk.setToZero()

    def homing(self, limit_pos=-270, target_pos=102):
        """Run homing calibration procedure.

        Args:
            limit_pos (float): limit angle to reach the stops (in degrees)
            target_pos (float): zero position relative to the stops (in degrees)
        """
        recent_speed = deque([], 10)

        for d in self.disks:
            d.setToZero()
        time.sleep(0.1)

        self.compliant = False
        time.sleep(0.1)

        trajs = self.goto(
            [limit_pos] * 3,
            duration=4,
            interpolation_mode='minjerk',
            wait=False,
        )

        for d in self.disks:
            d.rot_speed = True

        time.sleep(1)

        while True:
            recent_speed.append([d.rot_speed for d in self.disks])
            avg_speed = np.mean(recent_speed, axis=0)

            if np.all(avg_speed >= 0):
                for traj in trajs:
                    traj.stop()
                    traj.wait()
                break

            time.sleep(0.01)

        for d in self.disks:
            d.setToZero()

        for d in self.disks:
            d.rot_speed = False

        time.sleep(1)

        self.goto(
            [target_pos] * 3,
            duration=2,
            wait=True,
            interpolation_mode='minjerk',
        )

        for d in self.disks:
            d.setToZero()
        time.sleep(0.1)

        self.model.reset_last_angles()
        self.orient(Quaternion(1, 0, 0, 0), duration=1, wait=True)
