# two objects allocated at the same location (here, by two calls of the same
# function) are represented by the same weak site: all the references to the
# first one are updated when the second one is allocated
import _sv_verifier


def dec(f):
    def inner(x):
        return f(x)
    return inner


def a(x):
    return 1


def b(x):
    return "s"


fa = dec(a)
fb = dec(b)
g = fa if _sv_verifier.nondet_bool() else fb
r = g(0)
if isinstance(r, int):
    # reachable when g is fa
    assert False


class Box:
    pass


def mk(v):
    o = Box()
    o.v = v
    return o


x = mk(1)
y = mk("s")
# x.v is 1, but the two boxes are summarized: the assertion might fail
assert x.v == 1
