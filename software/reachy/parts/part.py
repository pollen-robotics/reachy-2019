from .motor import DynamixelMotor, OrbitaActuator
from .kinematic import Link, Chain


class ReachyPart(object):
    def __init__(self, name):
        self.name = name

    def attach_dxl_motors(self, luos_io, dxl_motors):
        self.motors = []

        for motor_name, config in dxl_motors.items():
            m = DynamixelMotor(self, motor_name, luos_io.find_dxl(config['id']), config)
            setattr(self, motor_name, m)
            self.motors.append(m)

    def attach_orbita_actuator(self, name, luos_io, config):
        luos_disks_motor = luos_io.find_orbital_disks()
        orb = OrbitaActuator(self, name, luos_disks_motor, config)
        setattr(self, name, orb)

    def attach_kinematic_chain(self, dxl_motors):
        self.kin_chain = Chain([
            Link(m['link-translation'], m['link-rotation'])
            for m in dxl_motors.values()
        ])
