from .arm import LeftArm, RightArm # noqa

try:
    from .head import Head # noqa
except ImportError:
    class Head(object):
        def __init__(self, *args, **kwargs):
            raise NotImplementedError('Install opencv if you want to use head!')
