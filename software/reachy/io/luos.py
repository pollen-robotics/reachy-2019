"""Wrapper module on top of pyluos Robot object."""
import time
import logging

from glob import glob

from pyluos import Device as LuosDevice
from pyluos.modules import DynamixelMotor

from ..error import LuosModuleNotFoundError, LuosGateNotFoundError
from .io import IO

logger = logging.getLogger(__name__)


def attempt_luos_connection(port, trials=5):
    """Try to connect to a Luos Gate."""
    io = LuosDevice(port, log_conf='')
    gate_name = io.modules[0].alias

    if trials > 0 and gate_name in ('r_right_arm', 'r_left_arm'):
        # TEMP: check if the dxl did respond
        if len(io.modules) < 3:
            io.close()
            time.sleep(0.1)
            return attempt_luos_connection(port, trials-1)

    return io


class SharedLuosIO(IO):
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

        if len(available_ports) == 0:
            return cls(port_template)

        for p in available_ports:
            io = cls(p)

            if io.gate_name == name:
                logger.info(f'Found gate "{io.gate_name}" on port "{p}"')
                return io
        else:
            raise LuosGateNotFoundError(f'Gate "{name}" not found on ports "{port_template}"')

    @classmethod
    def close_all_cached_gates(cls):
        """Close all connections to the Luos gate."""
        for io in SharedLuosIO.opened_io.values():
            io.close()
        SharedLuosIO.opened_io.clear()

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

    def find_dxl(self, dxl_name, dxl_config):
        """Retrieve a specified Dynamixel motor on the IO given its id.

        Args:
            dxl_config (dict): configuration of the searched dynamixel (including its id)
        """
        module_name = 'dxl_{}'.format(dxl_config['id'])

        m = self.find_module(module_name)

        if not isinstance(m, DynamixelMotor):
            raise LuosModuleNotFoundError(f'Wrong module type found for module "{module_name}" on bus "{self.port}"')

        return m

    def find_fan(self, fan_name):
        """Get a specific fan from its name."""
        return Fan(fan_name, self.find_module(fan_name))

    def find_orbita_disks(self):
        """Retrieve the three Luos modules controlling each Orbita disk."""
        return [
            OrbitaDisk(name, self.find_module(name))
            for name in ['disk_bottom', 'disk_middle', 'disk_top']
        ]

    def find_camera(self, camera_index):
        """Retrieve a camera."""
        # We import DualCamera here to avoid OpenCV ImportError
        # if we are not using the Head part.
        from .cam import BackgroundVideoCapture
        return BackgroundVideoCapture(camera_index)


class OrbitaDisk(object):
    """Orbita Disk Wrapper around luos controlled motor module."""

    def __init__(self, name, luos_disk) -> None:
        """Create a new Orbita disk using the luos module.

        Args:
            name (str): name of the disk (e.g. "disk_bottom").
            luos_disk: controlled_motor luos module

        """
        self.name = name
        self.luos_disk = luos_disk
        self.offset = 0

    def __repr__(self) -> str:
        """Get the OrbitaDisk string representation."""
        return f'<Orbita "{self.name}" pos="{self.rot_position}>'

    def setup(self):
        """Prepare the luos disk before controlling it.

        Enable position control, retrieve position and temperature.
        """
        self.luos_disk.rot_position_mode = True
        self.luos_disk.rot_position = True
        self.luos_disk.temperature = True

    @property
    def compliant(self):
        """Get the disk compliancy."""
        return self.luos_disk.compliant

    @compliant.setter
    def compliant(self, new_compliancy):
        """Set new compliancy (stiff/compliant)."""
        self.luos_disk.compliant = new_compliancy

    @property
    def rot_position(self):
        """Get the current angle position (in deg.)."""
        return self.luos_disk.rot_position - self.offset

    @property
    def target_rot_position(self):
        """Get the current target angle position (in deg.)."""
        return self.luos_disk.target_rot_position - self.offset

    @target_rot_position.setter
    def target_rot_position(self, new_pos):
        """Set a new target angle position (in deg.)."""
        self.luos_disk.target_rot_position = new_pos + self.offset

    @property
    def temperature(self):
        """Get the current motor temperature in C."""
        return self.luos_disk.temperature


class Fan(object):
    """Fan module for motor cooling."""

    def __init__(self, name, mod):
        """Create a new motor associated with its Luos module."""
        self.name = name
        self.mod = mod

    def __repr__(self):
        """Fan representation."""
        return f'<Fan "{self.name}" is "{self.status}">'

    @property
    def status(self):
        """Get the fan mode ('on' or 'off')."""
        return 'on' if self.mod.state else 'off'

    def on(self):
        """Turn the fan on."""
        self.mod.state = True

    def off(self):
        """Turn the fan off."""
        self.mod.state = False
