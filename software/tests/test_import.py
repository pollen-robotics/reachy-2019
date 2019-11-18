import unittest


class ImportTestCase(unittest.TestCase):
    def test_import(self):
        import reachy  # noqa: F401
        from reachy import Reachy  # noqa: F401

    def test_has_version(self):
        from reachy import __version__  # noqa: F401

    def test_import_parts(self):
        from reachy import parts  # noqa: F401
