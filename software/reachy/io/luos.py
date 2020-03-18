"""Wrapper module on top of pyluos Robot object."""
import time
import logging

from glob import glob

from pyluos import Robot as LuosIO
from pyluos.modules import DynamixelMotor

from ..error import LuosModuleNotFoundError, LuosGateNotFoundError


logger = logging.getLogger(__name__)


def attempt_luos_connection(port, trials=5):
    """Try to connect to a Luos Gate."""
    io = LuosIO(port, log_conf='')
    gate_name = io.modules[0].alias

    if trials > 0 and gate_name in ('r_right_arm', 'r_left_arm'):
        # TEMP: check if the dxl did respond
        if len(io.modules) < 3:
            io.close()
            time.sleep(0.1)
            return attempt_luos_connection(port, trials-1)

    return io


class SharedLuosIO(object):
    """
    Abstraction class for pyluos Robot object. Create a new connection with a Luos gate.

    Args:
        luos_port (str): name of the serial port used (e.g. '/dev/ttyUSB0')

    .. note:: If a connection on the same port already exists, the same IO will be used.

    The class is reponsible for holding active connections with Luos gate. A same gate can be shared among multiple IOs.
    """

    opened_io = {}

    def __init__(self, luos_port):
        """Create a new connection with a Luos gate."""
        if luos_port not in SharedLuosIO.opened_io:
            io = attempt_luos_connection(luos_port)
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

    def __repr__(self):
        """Shared IO representation."""
        mod = [m.alias for m in self.shared_io.modules]
        return f'<SharedLuosIO "port": "{self.port}" "modules": {mod}>'

    @classmethod
    def with_gate(cls, name, port_template):
        """Open a connection on the specified Luos gate.

        Args:
            name (str): name (or alias) of the searched Luos gate.
            port_template (str): template name for the possible serial ports (e.g. '/dev/ttyUSB*')

        """
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
            raise LuosGateNotFoundError(f'Gate "{name}" not found on ports "{port_template}"')

    @property
    def gate_name(self):
        """Retrieve the name of the Luos gate."""
        return self.shared_io.modules[0].alias

    def close(self):
        """Close the IO.

        .. warning:: You are responsible for handling correctly closing if you are using multiple connections on the same IO.
        """
        self.shared_io.close()
        logger.info('Luos IO connection closed', extra={
            'gate_name': self.gate_name,
            'port': self.port,
        })

    def find_module(self, module_name):
        """Retrieve a specified Luos module on the IO given its name.

        Args:
            module_name (str): name (or alias) of the researched module
        """
        try:
            return getattr(self.shared_io, module_name)
        except AttributeError:
            raise LuosModuleNotFoundError(
                message=f'Could not find module "{module_name}" on bus "{self.port}"',
                missing_module=module_name,
            )

    def find_dxl(self, dxl_id):
        """Retrieve a specified Dynamixel motor on the IO given its id.

        Args:
            dxl_id (int): ID of the researched dynamixel motor
        """
        module_name = 'dxl_{}'.format(dxl_id)

        m = self.find_module(module_name)

        if not isinstance(m, DynamixelMotor):
            raise LuosModuleNotFoundError(f'Wrong module type found for module "{module_name}" on bus "{self.port}"')

        return m

    def find_orbital_disks(self):
        """Retrieve the three Luos modules controlling each Orbita disk."""
        return [
            self.find_module(name)
            for name in ['disk_bottom', 'disk_middle', 'disk_top']
        ]
