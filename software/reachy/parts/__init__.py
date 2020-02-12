"""Reachy parts submodule.

Used to define all parts in Reachy:
* RightArm, LeftArm
* ForceGripper
* Head
"""

from .arm import LeftArm, RightArm # noqa

try:
    from .head import Head # noqa
except ImportError:
    class Head(object):
        """Placeholder Head part if OpenCV not found."""

        def __init__(self, *args, **kwargs):
            """Raise Error message if OpenCV not installed."""
            raise NotImplementedError('Install opencv if you want to use head!')
