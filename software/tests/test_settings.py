import os
import importlib


class TestSettings:
    def setup_method(self, method):
        self.conf = importlib.import_module('reachy.conf')
        self.settings = self.conf.settings

    def test_global_settings(self):
        assert self.settings.LOGGING == {}
        assert self.settings.REACHY_HARDWARE_SPECIFIC_SETTINGS

    def test_overload_settings(self):
        os.environ.setdefault('REACHY_SETTINGS_MODULE', 'mockup_settings')
        importlib.reload(self.conf)
        custom_settings = self.conf.settings

        assert custom_settings.LOGGING['dummy'] == 42
