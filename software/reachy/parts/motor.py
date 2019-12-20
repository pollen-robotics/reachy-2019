import time
import logging
import numpy as np

from threading import Timer

from pyquaternion import Quaternion
from orbita import Actuator as OrbitaModel

from ..trajectory.interpolation import Linear, MinimumJerk


logger = logging.getLogger(__name__)


class LuosDynamixelMotor(object):
    def __init__(self, root_part, name, luos_motor, config):
        self._root_part = root_part
        self._name = name

        self._motor = luos_motor

        self._offset = config['offset']
        self._direct = config['orientation'] == 'direct'

    def __repr__(self):
        mode = 'compliant' if self.compliant else 'stiff'
        return f'<Motor "{self.name}" pos="{self.present_position}" mode="{mode}">'

    @property
    def name(self):
        return f'{self._root_part.name}.{self._name}'

    # Position
    @property
    def present_position(self):
        return self._as_local_pos(self._motor.rot_position)

    @property
    def goal_position(self):
        return self._as_local_pos(self._motor.target_rot_position)

    @goal_position.setter
    def goal_position(self, value):
        self._motor.target_rot_position = self._to_motor_pos(value)

    @property
    def offset(self):
        return self._offset

    def is_direct(self):
        return self._direct

    def _as_local_pos(self, pos):
        return (pos if self.is_direct() else -pos) - self.offset

    def _to_motor_pos(self, pos):
        return (pos + self.offset) * (1 if self.is_direct() else -1)

    # Speed
    @property
    def moving_speed(self):
        return self._motor.target_rot_speed

    @moving_speed.setter
    def moving_speed(self, value):
        self._motor.target_rot_speed = value

    # Compliancy
    @property
    def compliant(self):
        return self._motor.compliant

    @compliant.setter
    def compliant(self, value):
        self._motor.compliant = value

    @property
    def torque_limit(self):
        return self._motor.power_ratio_limit

    @torque_limit.setter
    def torque_limit(self, value):
        self._motor.power_ratio_limit = value

    # Temperature
    @property
    def temperature(self):
        return self._motor.temperature

    def goto(self,
             goal_position, duration,
             starting_point='present_position',
             wait=False, interpolation_mode='linear'):
        interpolations = {
            'linear': Linear,
            'minjerk': MinimumJerk,
        }

        if interpolation_mode not in interpolations.keys():
            available = tuple(interpolations.keys())
            raise ValueError(f'interpolation_mode should be one of {available}')

        traj_player = interpolations[interpolation_mode](getattr(self, starting_point), goal_position, duration)
        traj_player.start(self)

        if wait:
            traj_player.wait()

        return traj_player



class PypotDynamixelMotor(object):
    def __init__(self, root_part, name, pypot_motor):
        self._root_part = root_part
        self._name = name

        self._motor = pypot_motor

    def __repr__(self):
        mode = 'compliant' if self.compliant else 'stiff'
        return f'<Motor "{self.name}" pos="{self.present_position}" mode="{mode}">'

    @property
    def name(self):
        return f'{self._root_part.name}.{self._name}'

    # Position
    @property
    def present_position(self):
        return self._motor.present_position

    @property
    def goal_position(self):
        return self._motor.goal_position

    @goal_position.setter
    def goal_position(self, value):
        self._motor.goal_position = value

    # Speed
    @property
    def moving_speed(self):
        return self._motor.moving_speed

    @moving_speed.setter
    def moving_speed(self, value):
        self._motor.moving_speed = value

    # Compliancy
    @property
    def compliant(self):
        return self._motor.compliant

    @compliant.setter
    def compliant(self, value):
        self._motor.compliant = value

    @property
    def torque_limit(self):
        return self._motor.torque_limit

    @torque_limit.setter
    def torque_limit(self, value):
        self._motor.torque_limit = value

    # Temperature
    @property
    def temperature(self):
        return self._motor.present_temperature

    def goto(self,
             goal_position, duration,
             starting_point='present_position',
             wait=False, interpolation_mode='linear'):
        interpolations = {
            'linear': Linear,
            'minjerk': MinimumJerk,
        }

        if interpolation_mode not in interpolations.keys():
            available = tuple(interpolations.keys())
            raise ValueError(f'interpolation_mode should be one of {available}')

        traj_player = interpolations[interpolation_mode](getattr(self, starting_point), goal_position, duration)
        traj_player.start(self)

        if wait:
            traj_player.wait()

        return traj_player


class OrbitaActuator(object):
    def __init__(
        self, root_part, name, luos_disks_motor,
        Pc_z, Cp_z, R, R0,
        pid, reduction, wheel_size, encoder_res,
    ):
        self.disk_bottom, self.disk_middle, self.disk_top = luos_disks_motor
        self.model = OrbitaModel(Pc_z=Pc_z, Cp_z=Cp_z, R=R, R0=R0.tolist())

        self._compliancy = False
        self._moving_speed = 50

        self.setup(
            pid=pid,
            reduction=reduction,
            wheel_size=wheel_size,
            encoder_res=encoder_res,
            moving_speed=self.moving_speed,
        )

    @property
    def disks(self):
        return [self.disk_top, self.disk_middle, self.disk_bottom]

    @property
    def compliant(self):
        return self._compliancy

    @compliant.setter
    def compliant(self, compliancy):
        self._compliancy = compliancy

        for d in self.disks:
            d.compliant = compliancy

    @property
    def moving_speed(self):
        return self._moving_speed

    @moving_speed.setter
    def moving_speed(self, speed):
        self._moving_speed = speed

        for d in self.disks:
            d.target_rot_speed = speed

    def point_at(self, vector, angle):
        thetas = self.model.get_angles_from_vector(vector, angle)

        for d, q in zip(self.disks, thetas):
            q = -q  # FIXME: Temporary path due to reversed encoder
            d.target_rot_position = q

    def orient(self, quat, duration=-1, wait=False):
        thetas = self.model.get_angles_from_quaternion(quat.w, quat.x, quat.y, quat.z)
        # FIXME: Temporary patch due to reversed encoder
        thetas = [-q for q in thetas]

        if duration > 0:
            speeds = [
                abs(q - d.rot_position) / duration
                for d, q in zip(self.disks, thetas)
            ]
            for d, s in zip(self.disks, speeds):
                d.target_rot_speed = s

        for d, q in zip(self.disks, thetas):
            d.target_rot_position = q

        if wait:
            time.sleep(duration)

    def find_quaternion_transform(model, v_origin, v_target):
        v_origin = np.array(v_origin)
        v_origin = v_origin / np.linalg.norm(v_origin)

        v_target = np.array(v_target)
        v_target = v_target / np.linalg.norm(v_target)

        V = np.cross(v_origin, v_target)
        V = V / np.linalg.norm(V)

        alpha = np.arccos(np.vdot(v_origin, v_target))
        if alpha == 0:
            return Quaternion(1, 0, 0, 0)

        return Quaternion(axis=V, radians=alpha)

    def setup(self, pid, reduction, wheel_size, encoder_res, moving_speed):
        for disk in self.disks:
            disk.limit_current = 0.4
            disk.encoder_res = encoder_res
            disk.reduction = reduction
            disk.wheel_size = wheel_size
            disk.positionPid = pid
            disk.rot_position_mode = True
            disk.rot_speed_mode = True
            disk.rot_position = True
            disk.rot_speed = True
            disk.setToZero()

            # FIXME: temporary fix (see https://github.com/Luos-Robotics/pyluos/issues/53)
            time.sleep(0.1)
            disk.target_rot_speed = moving_speed

    def start_timer(self):
        if not hasattr(self, '_temp_timer'):
            self._temp_timer = None

        if (self._temp_timer is None) or (not self._temp_timer.is_alive()):
            def f():
                logger.warning('Timer sets orbita compliant!')
                self.compliant = True
                self._temp_timer = None

            self._temp_timer = Timer(60, f)
            self._temp_timer.start()

    def cancel_timer(self):
        if not hasattr(self, '_temp_timer'):
            self._temp_timer = None

        if self._temp_timer is not None:
            self._temp_timer.cancel()
            self._temp_timer = None
