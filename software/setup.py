#!/usr/bin/env python

import re

from setuptools import setup, find_packages
from os import path


def version():
    with open('reachy/_version.py') as f:
        _, version = f.readlines()
        return re.search(r"^__version__ = ['\"]([^'\"]*)['\"]", version).group(1)


here = path.abspath(path.dirname(__file__))

with open(path.join(here, '..', 'README.md'), encoding='utf-8') as f:
    long_description = f.read()


setup(
    name='reachy',
    version=version(),
    packages=find_packages(exclude=['tests']),

    package_data={'reachy': ['*.npy']},

    install_requires=[
        'pyluos==1.2.4',
        'numpy',
        'scipy>=1.4.0',
        'orbita>=0.3.1',
        'pyquaternion',
        'websockets',
        'Pillow',
    ],

    extras_require={
        'head': ['opencv-python'],
        'log': ['zzlog'],

        'doc': ['sphinx', 'sphinx-autoapi'],
        'lint': ['flake8', 'pydocstyle'],
        'test': ['pytest'],
    },

    entry_points={
        'console_scripts': [
            'dxl-config=reachy.utils.dxl_config:main',
            'orbita-config=reachy.utils.orbita_config:main',
            'reachy-setup-motorlimits=reachy.utils.setup_angle_limits:main',
            'orbita-zero=reachy.utils.orbita_zero:main',
        ],
    },

    author='Pollen Robotics',
    author_email='contact@pollen-robotics.com',
    url='https://github.com/pollen-robotics/reachy',

    description='Open source interactive robot to explore real-world applications!',
    long_description=long_description,
    long_description_content_type='text/markdown',
)
