"""Part abstraction module."""

import numpy as np

from .motor import DynamixelMotor, OrbitaActuator
from .kinematic import Link, Chain


class ReachyPart(object):
    """Part abstraction class.

    Args:
        name (str): name of the new part, can be composed if it's a subpart (eg. right_arm.hand)

    Define instantiation, teardown functionalities.
    Also provides attach function for dynamixel motors and orbita actuator.
    """

    def __init__(self, name):
        """Create a new part."""
        self.name = name
        self.motors = []

    def teardown(self):
        """Clean up before closing."""
        pass

    def attach_dxl_motors(self, luos_io, dxl_motors):
        """Attach given dynamixel motors to a part.

        Args:
            luos_io (reachy.io.SharedLuosIO): io to the Luos gate where the motors are connected
            dxl_motors (dict): motors config, the config must at least include an id for each motor (see attach_kinematic_chain for extra parameters)
        """
        self.motors = []

        for motor_name, config in dxl_motors.items():
            m = DynamixelMotor(self, motor_name, luos_io.find_dxl(config['id']), config)
            setattr(self, motor_name, m)
            self.motors.append(m)

    def create_orbita_actuator(self, name, luos_io, config):
        """Attach an orbita actuator to a part.

        Args:
            name (str): name of the orbita actuator (eg neck for the head part)
            luos_io (reachy.io.SharedLuosIO): io to the Luos gate where the Orbita disk controller are connected
            config (dict): orbita configuration (see OrbitaActuator for details)
        """
        luos_disks_motor = luos_io.find_orbital_disks()
        orb = OrbitaActuator(self, name, luos_disks_motor, **config)
        setattr(self, name, orb)
        return orb

    def attach_kinematic_chain(self, dxl_motors):
        """Attach a kinematic chain composed of the given motors.

        Args:
            dxl_motors (dict): dynamixel motors config (as given in attach_dxl_motors), the config should also include 'link-translation' and 'link-rotation' for each motor
        """
        def compute_bounds(m):
            bounds = [
                np.deg2rad(a + m['offset']) * (1 if m['orientation'] == 'direct' else -1)
                for a in m['angle-limits']
            ]
            lb = min(bounds)
            rb = max(bounds)

            return (lb, rb)

        self.kin_chain = Chain([
            Link(m['link-translation'], m['link-rotation'], compute_bounds(m))
            for m in dxl_motors.values()
        ])
