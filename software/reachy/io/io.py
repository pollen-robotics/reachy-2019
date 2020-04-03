"""Abstract IO definition."""


class IO(object):
    """Abstract IO class.

    Defines how to find and interact with the module.
    """

    def find_module(self, module_name):
        """Get a specific module from the IO."""
        raise NotImplementedError

    def find_dxl(self, dxl_name, dxl_id):
        """Get a specific dynamixel motor from the IO."""
        raise NotImplementedError

    def find_orbital_disks(self):
        """Get a specific orbital disk from the IO."""
        raise NotImplementedError

    def close(self):
        """Close and clean the IO."""
        raise NotImplementedError
