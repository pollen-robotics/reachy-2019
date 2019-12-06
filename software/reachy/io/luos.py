import logging

from glob import glob

from pyluos import Robot as LuosIO
from pyluos.modules import DynamixelMotor

logger = logging.getLogger(__name__)


class SharedLuosIO(object):
    opened_io = {}

    def __init__(self, luos_port):
        if luos_port not in SharedLuosIO.opened_io:
            io = LuosIO(luos_port, log_conf='')
            SharedLuosIO.opened_io[luos_port] = io

            logger.info('Connected to new Luos IO', extra={
                'luos_port': luos_port,
                'gate_name': io.modules[0].alias,
                'modules': [mod.alias for mod in io.modules],
            })
            # FIXME: wait for a first sync of all modules
            import time
            time.sleep(1)
        self.shared_io = SharedLuosIO.opened_io[luos_port]
        self.port = luos_port

    @classmethod
    def with_gate(cls, name, port_template):
        logger.info(f'Looking for gate "{name}" on ports "{port_template}"')

        available_ports = glob(port_template)
        if len(available_ports) == 1:
            return cls(available_ports[0])

        for p in available_ports:
            io = cls(p)

            if io.gate_name == name:
                logger.info(f'Found gate "{io.gate_name}" on port "{p}"')
                return io
        else:
            return cls(port_template)

    @property
    def gate_name(self):
        return self.shared_io.modules[0].alias

    def close(self):
        self.shared_io.close()
        logger.info('Luos IO connection closed', extra={
            'gate_name': self.gate_name,
            'port': self.port,
        })

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
