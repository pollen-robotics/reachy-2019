"""Part abstraction module."""

import numpy as np

from .motor import DynamixelMotor, OrbitaActuator
from .kinematic import Link, Chain

from ..io import IO, luos, ws


class ReachyPart(object):
    """Part abstraction class.

    Args:
        name (str): name of the new part, can be composed if it's a subpart (eg. right_arm.hand)
        io (str): port name where the modules can be found

    Define instantiation, teardown functionalities.
    Also provides attach function for dynamixel motors and orbita actuator.
    """

    def __init__(self, name, io):
        """Create a new part."""
        self.name = name
        self.motors = []

        if isinstance(io, IO):
            self.io = io
        elif isinstance(io, str) and io == 'ws':
            self.io = ws.WsIO.shared_server(self.name)
        else:
            gate_name = self.name.split('.')[0]
            gate_name = f'r_{gate_name}'
            self.io = luos.SharedLuosIO.with_gate(gate_name, io)

    def teardown(self):
        """Clean up before closing."""
        self.io.close()

    def attach_dxl_motors(self, dxl_motors):
        """Attach given dynamixel motors to a part.

        Args:
            dxl_motors (dict): motors config, the config must at least include an id for each motor (see attach_kinematic_chain for extra parameters)
        """
        self.motors = []

        for motor_name, config in dxl_motors.items():
            m = DynamixelMotor(self, motor_name, self.io.find_dxl(motor_name, config), config)
            setattr(self, motor_name, m)
            self.motors.append(m)

    def create_orbita_actuator(self, name, config):
        """Attach an orbita actuator to a part.

        Args:
            name (str): name of the orbita actuator (eg neck for the head part)
            config (dict): orbita configuration (see OrbitaActuator for details)
        """
        disks_motor = self.io.find_orbita_disks()
        orb = OrbitaActuator(self, name, disks_motor, **config)
        setattr(self, name, orb)
        return orb

    def attach_kinematic_chain(self, dxl_motors):
        """Attach a kinematic chain composed of the given motors.

        Args:
            dxl_motors (dict): dynamixel motors config (as given in attach_dxl_motors), the config should also include 'link-translation' and 'link-rotation' for each motor
        """
        def compute_bounds(m):
            pos = np.array(m['angle-limits'])
            bounds = (pos if m['orientation'] == 'direct' else -pos) - m['offset']
            bounds = np.deg2rad(bounds)
            lb = min(bounds)
            rb = max(bounds)

            return (lb, rb)

        self.kin_chain = Chain([
            Link(m['link-translation'], m['link-rotation'], compute_bounds(m))
            for m in dxl_motors.values()
        ])
