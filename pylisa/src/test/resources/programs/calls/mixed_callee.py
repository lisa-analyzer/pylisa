import sys
import mystery


def g():
    return 1


f = g if len(sys.argv) > 1 else mystery.maker
x = f()  # @mixed
