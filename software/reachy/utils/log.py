import sys
import logging

from datetime import datetime

from pythonjsonlogger import jsonlogger

import reachy


class ReachyJsonFormatter(jsonlogger.JsonFormatter):
    def add_fields(self, log_record, record, message_dict):
        jsonlogger.JsonFormatter.add_fields(self, log_record, record, message_dict)

        log_record['level'] = record.levelname

        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        log_record['timestamp'] = now


def setup(filename, level):
    logger = logging.getLogger(reachy.__name__)
    logger.setLevel(level)

    handler = logging.FileHandler(filename)

    formatter = ReachyJsonFormatter()
    handler.setFormatter(formatter)

    logger.addHandler(handler)

    default_hook = sys.excepthook

    def log_exception(exc_type, exc_value, exc_traceback):
        if not issubclass(exc_type, KeyboardInterrupt):
            logger.error('Uncaught exception', exc_info=(exc_type, exc_value, exc_traceback))
        default_hook(exc_type, exc_value, exc_traceback)

    sys.excepthook = log_exception
