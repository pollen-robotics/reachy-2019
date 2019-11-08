import logging

from glob import glob

from pyluos import Robot as LuosIO
from pyluos.modules import DynamixelMotor

logger = logging.getLogger(__name__)


def find_gate_name(port):
    r = LuosIO(port, log_conf='')
    name = r.modules[0].alias
    r.close()
    return name


class SharedLuosIO(object):
    opened_io = {}

    def __init__(self, luos_port):
        if luos_port not in SharedLuosIO.opened_io:
            logger.info('Connecting to new Luos IO', extra={
                'luos_port': luos_port,
            })
            SharedLuosIO.opened_io[luos_port] = LuosIO(luos_port, log_conf='')
            # FIXME: wait for a first sync of all modules
            import time
            time.sleep(1)
        self.shared_io = SharedLuosIO.opened_io[luos_port]
        self.port = luos_port

    @classmethod
    def with_gate(cls, name, port_template):
        for p in glob(port_template):
            gate_name = (
                find_gate_name(p) if p not in SharedLuosIO.opened_io
                else SharedLuosIO.opened_io[p].modules[0].alias
            )

            if gate_name == name:
                return cls(p)

    def find_module(self, module_name):
        try:
            return getattr(self.shared_io, module_name)
        except AttributeError:
            raise IOError(f'Could not find module "{module_name}" on bus "{self.port}"')

    def find_dxl(self, dxl_id):
        module_name = 'dxl_{}'.format(dxl_id)

        m = self.find_module(module_name)

        if not isinstance(m, DynamixelMotor):
            raise IOError(f'Wrong module type found for module "{module_name}" on bus "{self.port}"')

        return m

    def find_orbital_disks(self):
        return [
            self.find_module(name)
            for name in ['disk_bottom', 'disk_middle', 'disk_top']
        ]
