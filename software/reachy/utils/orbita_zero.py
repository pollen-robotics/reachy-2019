"""Retrieve Orbita Zero hardware and write it to the specified config file.

This tool must be run while Orbita is fixed at the Zero Position using a specific 3D part.

"""

import time
import shutil
import argparse

from glob import glob
from pathlib import Path

from pyluos import Device
from reachy.conf import settings


def _write_hardware_zero(zero, filename):
    with open(filename) as f:
        lines = f.readlines()

    zero = list(zero)

    config = []
    for line in lines:
        if line.startswith('ORBITA_NECK_HARDWARE_ZERO = ['):
            config.append(f'ORBITA_NECK_HARDWARE_ZERO = {zero}\n')
        else:
            config.append(line)

    with open(filename, 'w') as f:
        f.write(''.join(config))


def main():
    """Get the Zero hardware from Orbita."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--luos_port', default='/dev/ttyUSB*', help='Orbita gate luos port (default: %(default)s)')
    args = parser.parse_args()

    ports = glob(args.luos_port)
    if len(ports) == 0:
        ports = [args.luos_port]

    device = Device(ports[0])
    time.sleep(0.5)

    hardware_zero = (
        device.disk_top.rot_position,
        device.disk_middle.rot_position,
        device.disk_bottom.rot_position,
    )

    print(f'Find Orbita Hardware Zero at {hardware_zero}')

    filename = Path({settings.REACHY_HARDWARE_SPECIFIC_SETTINGS}).expanduser()
    bkp_file = Path(f'{filename}.bkp')

    print(f'Making backup file at {bkp_file}')
    shutil.copy(filename, bkp_file)

    print(f'Value will be store in {filename}')
    _write_hardware_zero(hardware_zero, filename)


if __name__ == '__main__':
    main()
