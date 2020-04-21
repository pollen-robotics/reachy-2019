"""Abstract IO definition."""


class IO(object):
    """Abstract IO class.

    Defines how to find and interact with the module.
    """

    def find_module(self, module_name):
        """Get a specific module from the IO."""
        raise NotImplementedError

    def find_dxl(self, dxl_name, dxl_config):
        """Get a specific dynamixel motor from the IO."""
        raise NotImplementedError

    def find_orbita_disks(self):
        """Get a specific orbita disk from the IO."""
        raise NotImplementedError

    def close(self):
        """Close and clean the IO."""
        raise NotImplementedError
