class ReachyPart(object):
    def __init__(self, name):
        self.name = name

    def attach_dxl_motors(self, luos_io, dxl_motors):
        self.motors = []

        for name, id in dxl_motors.items():
            m = luos_io.find_dxl(id)
            setattr(self, name, m)
            self.motors.append(m)
