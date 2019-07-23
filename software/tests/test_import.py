import unittest


class ImportTestCase(unittest.TestCase):
    def test_import(self):
        import reachy

    def test_has_version(self):
        from reachy import __version__
