import sys


def g():
    return 1


f = g if len(sys.argv) > 1 else 5
x = f()  # @partly
