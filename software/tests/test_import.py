import unittest


class ImportTestCase(unittest.TestCase):
    def test_import(self):
        import reachy
        from reachy import Reachy

    def test_has_version(self):
        from reachy import __version__

    def test_import_parts(self):
        from reachy import parts
