import sys
import testnatives


def make(cls):
    return object.__new__(cls)


class C:
    __new__ = testnatives.nothing if len(sys.argv) > 1 else make

    def __init__(self):
        self.tag = 1


c = C()  # @maybe_none
