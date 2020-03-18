"""Discovery utility tools.

Provide automatic detection mechanisms to check which parts are present on your robot and their status.
.. warning:: Make sure that no other Python instance is connected to the robot before running those functions.
"""

from reachy import parts
from reachy.error import LuosModuleNotFoundError, LuosGateNotFoundError, CameraNotFoundError


def discover_head(luos_port='/dev/ttyUSB*', camera_id=0):
    """Check if an head part is connected to a Reachy."""
    part_name = 'head'

    try:
        head = parts.Head(
            luos_port=luos_port,
            camera_id=camera_id,
        )
        head.teardown()
        status = 'ok'
        details = ''

    except LuosGateNotFoundError as e:
        status = 'missing'
        details = str(e)

    except LuosModuleNotFoundError as e:
        status = 'problem'
        details = f'module "{e.missing_module}" missing'

    except CameraNotFoundError as e:
        status = 'problem'
        details = f'camera {e.camera_id} missing'

    return {
        'part_name': part_name,
        'status': status,
        'details': details,
    }


def discover_arm(side, luos_port='/dev/ttyUSB*', hand='force_gripper'):
    """Check if an arm part is connected to a Reachy."""
    Arm = parts.LeftArm if side == 'left' else parts.RightArm

    part_name = f'{side}_arm'

    try:
        arm = Arm(
            luos_port=luos_port,
            hand=hand,
        )
        arm.teardown()
        status = 'ok'
        details = ''

    except LuosGateNotFoundError:
        status = 'missing'
        details = ''

    except LuosModuleNotFoundError as e:
        status = 'problem'
        details = f'module "{e.missing_module}" missing'

    return {
        'part_name': part_name,
        'status': status,
        'details': details,
    }


def discover_all():
    """Check which part is connected to a Reachy."""
    return [
        discover_arm(side='left'),
        discover_head(),
        discover_arm(side='right'),
    ]


def main():
    """Check which part is connected to a Reachy."""
    for part in discover_all():
        name = part['part_name']
        status = part['status']
        details = part['details']

        if status == 'ok':
            message = f'Part "{name}" was found and everything seems ok!'

        elif status == 'missing':
            message = f'Part "{name}" is missing, check connections if it should have been found.'

        elif status == 'problem':
            message = f'Part "{name}" was found but it seems there is a problem: {details}!!!'
        print(message)


if __name__ == '__main__':
    main()
