from spherical_joint import Actuator as OrbitaModel

from ..trajectory.interpolation import Linear, MinimumJerk


class DynamixelMotor(object):
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


class OrbitalActuator(object):
    def __init__(self, root_part, name, luos_disks_motor, config):
        self.disk_bottom, self.disk_middle, self.disk_top = luos_disks_motor
        self.model = OrbitaModel(**config)

        self.setup()

    @property
    def disks(self):
        return [self.disk_top, self.disk_middle, self.disk_bottom]

    def turn_compliant(self):
        for disk in self.disks:
            disk.compliant = True

    def turn_stiff(self):
        for disk in self.disks:
            disk.compliant = False

    def goto_zero(self):
        for disk in self.disks:
            disk.target_rot_position = 0

    def point_at(self, vector, angle):
        thetas = self.model.get_angles_from_vector(vector, angle)
        for d, q in zip(self.disks, thetas):
            d.target_rot_position = q

    def orient(self, quat):
        thetas = self.model.get_angles_from_quaternion(*quat)
        for d, q in zip(self.disks, thetas):
            d.target_rot_position = q

    def setup(self):
        pid = [
            [9, 0.02, 120],
            [9, 0.06, 110],
            [9, 0.06, 80],
        ]

        for i, disk in enumerate(self.disks):
            disk.rot_position = False
            disk.setToZero()
            disk.reduction = 214
            disk.encoder_res = 5
            disk.wheel_size = 79
            disk.positionPid = pid[i]
            disk.rot_position_mode = True
            disk.rot_position = True
