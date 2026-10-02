import argparse


def require(s):
    if s is None:
        raise ValueError("none")
    return s


def validate(s):
    return require(s)


def check(s):
    return require(s)


def setup(p):
    p.add_argument("--k", type=check)


p = argparse.ArgumentParser()
p.add_argument("--n", type=lambda s: validate(s))
hooks = [setup]
args = p.parse_args()
c = input()
if c == 'a':
    validate(None)  # @validate
if c == 'b':
    check(None)  # @check


def deep(s):
    return require(s)


def hook():
    deep(None)


handlers = [hook]
if c == 'd':
    deep(None)  # @deep
