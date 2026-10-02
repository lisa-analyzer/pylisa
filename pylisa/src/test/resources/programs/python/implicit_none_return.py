import sys


def setup():
    x = 1


def maybe(c):
    if c:
        return 1


def bare():
    return


r = setup()  # @r
if r is None:
    y = 1  # @none
else:
    y = 2  # @other
m = maybe(len(sys.argv) > 1)  # @maybe
b = bare()  # @bare
