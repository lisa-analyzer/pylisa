import sys


def make():
    return 1


class Box:
    pass


k = make if len(sys.argv) > 1 else Box
v = k()  # @either
