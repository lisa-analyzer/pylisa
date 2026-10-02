import sys
import mystery


def g():
    return 1


def call(f):
    return f()  # @inside


f = g if len(sys.argv) > 1 else mystery.maker
a = call(f)
b = call(f)
