"""Reachy top-level module.

Exposes:

* :py:mod:`~reachy.__version__` number
* :py:class:`~Reachy` class
* :py:mod:`~reachy.parts` submodule
"""

from ._version import __version__  # noqa: F401
from .reachy import Reachy  # noqa: F401


def setup():
    from reachy.conf import settings
    from reachy.utils.log import configure_logging

    configure_logging(settings.LOGGING_CONFIG, settings.LOGGING)


setup()
