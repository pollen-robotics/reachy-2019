"""Custom errors for Reachy."""


class LuosModuleNotFoundError(Exception):
    """Luos Module missing."""

    def __init__(self, message, missing_module):
        """Define Luos Module Error.

        Args:
            message (str): custom error message
            missing_module (str): name of the missing luos module
        """
        super().__init__(message)

        self.missing_module = missing_module


class LuosGateNotFoundError(Exception):
    """Luos Gate missing."""


class CameraNotFoundError(Exception):
    """Camera missing."""

    def __init__(self, message, camera_id):
        """Define Camera missing Error.

        Args:
            message (str): custom error message
            camera_id (int): index of the missing camera
        """
        super().__init__(message)

        self.camera_id = camera_id
