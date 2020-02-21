"""Command line utility tool for configuring dynamixel motor.

This tool let you configure one of the dynamixel motor used in Reachy. It can be used when you have to change a broken motor for instance.
In more details, this tool will:

* change the id
* set the angle limits

.. warning:: Make sure to have only one dynamixel motor connected when you configure it!
"""

import time
import argparse

from contextlib import closing
from pyluos import Robot as LuosIO

from reachy.parts import RightArm, LeftArm, Head
from reachy.parts.hand import LeftForceGripper, RightForceGripper

reachy_config = {}
reachy_config.update({f'right_arm.{m}': c for m, c in RightArm.dxl_motors.items()})
reachy_config.update({f'right_arm.hand.{m}': c for m, c in RightForceGripper.dxl_motors.items()})
reachy_config.update({f'left_arm.{m}': c for m, c in LeftArm.dxl_motors.items()})
reachy_config.update({f'left_arm.hand.{m}': c for m, c in LeftForceGripper.dxl_motors.items()})
reachy_config.update({f'head.{m}': c for m, c in Head.dxl_motors.items()})


def main():
    """Main dxl-config entry point."""
    parser = argparse.ArgumentParser()
    parser.add_argument('luos_port')
    parser.add_argument('motor_name', choices=reachy_config.keys())
    args = parser.parse_args()

    name = args.motor_name
    conf = reachy_config[name]
    id = conf['id']
    limits = conf['angle-limits']

    with closing(LuosIO(args.luos_port)) as io:
        print(f'Will configure motor {name}...')

        dxl_motors_found = [m for m in io.modules if m.alias.startswith('dxl_')]

        if len(dxl_motors_found) == 0:
            raise EnvironmentError(f'No motors found on {args.luos_port}! Check the cable and try again.')

        if len(dxl_motors_found) > 1:
            raise EnvironmentError(f'{len(dxl_motors_found)} motors found on {args.luos_port}! Connect only the motor you want to configure and try again!')

        dxl_motor = dxl_motors_found[0]

        print(f'\t with id {id}...')
        dxl_motor.set_id(conf['id'])
        time.sleep(0.25)

        print(f'\t and limits {limits}...')
        dxl_motor.rot_position_limit = conf['angle_limit']

        time.sleep(0.25)
        print('Done!')


if __name__ == '__main__':
    main()
