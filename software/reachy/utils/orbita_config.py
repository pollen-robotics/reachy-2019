"""Configure gate and three motor modules for an Orbita Actuator.

This tool:
* change the name of the gate (eg. 'r_head')
* change the name of the controlled motor modules ('disk_top', 'disk_middle', 'disk_bottom')

"""

import time
import argparse

from contextlib import closing
from pyluos import Device as LuosDevice


def main():
    """Run main orbita config entry point."""
    parser = argparse.ArgumentParser()
    parser.add_argument('luos_port')
    parser.add_argument('part_name', choices=['head'])
    args = parser.parse_args()

    with closing(LuosDevice(args.luos_port)) as io:
        if len(io.modules) != 4:
            names = [m.alias for m in io.modules]
            raise ValueError(f'Incoherent modules found: {names}!')

        gate = io.modules[0]
        gate_name = f'r_{args.part_name}'
        if gate.alias != gate_name:
            print(f'Changing gate "{gate.alias}" name to "{gate_name}"')
            gate.rename(gate_name)
            time.sleep(0.5)

        for module, disk in zip(io.modules[1:], ['top', 'middle', 'bottom']):
            disk_name = f'disk_{disk}'
            if module.alias != disk_name:
                print(f'Changing module "{module.alias}" to "{disk_name}"')
                module.rename(disk_name)
                time.sleep(0.5)

    print('Everything is setup!')


if __name__ == '__main__':
    main()
