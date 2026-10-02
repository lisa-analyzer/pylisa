import sys


class C:
    def __init__(self, v):
        self.v = v


def g(v):
    return v


def make(k, v):
    return k(v)  # @make


make(C, 1)
make(C if len(sys.argv) > 1 else g, 1)
