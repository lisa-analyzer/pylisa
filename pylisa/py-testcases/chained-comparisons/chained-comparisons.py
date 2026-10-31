# a < b < c is a < b and b < c, where b is evaluated once
import _sv_verifier

calls = [0]


def f():
    calls[0] = calls[0] + 1
    return 3


t = _sv_verifier.nondet_int()
n = _sv_verifier.nondet_int()
if 0 < t < n:
    assert t >= 1
    assert n >= 2
b = 0 < f() < 5
assert calls[0] == 1
assert b
c = 1 < 0 < f()
assert calls[0] == 1
assert not c
