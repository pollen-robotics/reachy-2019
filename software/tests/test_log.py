import unittest
import logging
import json

from tempfile import NamedTemporaryFile

import reachy

from reachy.utils import log


class LogTestCase(unittest.TestCase):
    def setUp(self):
        self.f = NamedTemporaryFile()

        log.setup(
            filename=self.f.name,
            level='DEBUG',
        )

        self.logger = logging.getLogger(reachy.__name__)

    def test_log(self):
        message = 'Hello World!'
        self.logger.error(message)

        with open(self.f.name) as f:
            lines = f.readlines()
            logs = [json.loads(l) for l in lines]

        for l in logs:
            for k in ('message', 'level', 'timestamp'):
                keys = l.keys()
                self.assertTrue(k in keys, f'{k} not in {keys}')

        for l in logs:
            if l['level'] == 'ERROR' and l['message'] == message:
                break
        else:
            raise KeyError(f'log message not found in {logs}')
