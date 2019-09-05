from .motor import ReachyMotor


class ReachyPart(object):
    def __init__(self, name):
        self.name = name

    def attach_dxl_motors(self, luos_io, dxl_motors):
        self.motors = []

        for motor_name, config in dxl_motors.items():
            m = ReachyMotor(self, motor_name, luos_io.find_dxl(config['id']), config)
            setattr(self, motor_name, m)
            self.motors.append(m)
