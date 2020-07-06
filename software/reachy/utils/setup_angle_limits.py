"""Command line utility tool to setup all dynamixel motors angle limits in a Reachy robot.

.. warning:: Make sure the motor ids are correctly set!
"""

import time
import argparse

from reachy.parts import RightArm, LeftArm, Head
from reachy.parts.hand import LeftForceGripper, RightForceGripper
from reachy.error import LuosModuleNotFoundError, LuosGateNotFoundError


reachy_config = {}
reachy_config.update({f'right_arm.{m}': c for m, c in RightArm.dxl_motors.items()})
reachy_config.update({f'right_arm.hand.{m}': c for m, c in RightForceGripper.dxl_motors.items()})
reachy_config.update({f'left_arm.{m}': c for m, c in LeftArm.dxl_motors.items()})
reachy_config.update({f'left_arm.hand.{m}': c for m, c in LeftForceGripper.dxl_motors.items()})
reachy_config.update({f'head.{m}': c for m, c in Head.dxl_motors.items()})


def main():
    """Configure all dynamixel motor limits on a Reachy robot."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--luos_port', default='/dev/ttyUSB*')
    args = parser.parse_args()

    for Arm in (LeftArm, RightArm):
        part_name = Arm.__name__

        print(f'Connecting to part "{part_name}"...')
        try:
            arm = Arm(io=args.luos_port, hand='force_gripper')
            time.sleep(0.25)

            for m in arm.motors:
                limits = reachy_config[m.name]['angle-limits']
                print(f'\t Setting motor "{m.name}" limits to {limits}.')
                m._motor.rot_position_limit = tuple(limits)
                time.sleep(0.25)

        except LuosModuleNotFoundError as e:
            print(f'Could not connect to {part_name}')
            print(f'Error: {e}')
        except LuosGateNotFoundError:
            print(f'No {part_name} found, skipping.')


if __name__ == '__main__':
    main()
