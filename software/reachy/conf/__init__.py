import os
import importlib

from pathlib import Path

ENVIRONMENT_VARIABLE = "REACHY_SETTINGS_MODULE"


class Settings:
    def __init__(self):
        self.inject_settings('reachy.conf.global_settings')

        settings_module = os.environ.get(ENVIRONMENT_VARIABLE)
        if settings_module:
            self.inject_settings(settings_module)

        path = Path(self.REACHY_HARDWARE_SPECIFIC_SETTINGS).expanduser()
        spec = importlib.util.spec_from_file_location('reachy.conf.hardware', path)
        hardware_settings_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(hardware_settings_module)

        self.hardware = HardwareSettings(hardware_settings_module)

    def inject_settings(self, settings_module_name):
        settings_module = importlib.import_module(settings_module_name)

        for setting in dir(settings_module):
            if setting.isupper():
                setattr(self, setting, getattr(settings_module, setting))


class HardwareSettings:
    def __init__(self, hardware_settings_module):
        for setting in dir(hardware_settings_module):
            if setting.isupper:
                setattr(self, setting, getattr(hardware_settings_module, setting))


settings = Settings()
