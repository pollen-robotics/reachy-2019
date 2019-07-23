from pyluos import Robot as LuosIO


class SharedLuosIO(object):
    opened_io = {}

    def __init__(self, luos_port):
        if luos_port not in SharedLuosIO.opened_io:
            SharedLuosIO.opened_io[luos_port] = LuosIO(luos_port)
        self.shared_io = SharedLuosIO.opened_io[luos_port]
        self.port = luos_port

    def find_module(self, module_name):
        try:
            return getattr(self.shared_io, module_name)
        except AttributeError:
            raise IOError('Could not find module "{}" on bus "{}"'.format(module_name, self.port))
