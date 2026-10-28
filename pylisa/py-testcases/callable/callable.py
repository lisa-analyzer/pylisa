# callable(obj) holds for functions, classes, and the objects whose class
# defines __call__
import _sv_verifier


def f():
    return 1


class C:
    pass


class K:
    def __call__(self):
        return 2


assert callable(f)
assert callable(C)
assert callable(K())
assert callable(len)
assert callable(lambda: 3)
assert not callable(1)
assert not callable("s")
assert not callable([1])
assert not callable(C())
if _sv_verifier.nondet_bool():
    # AssertionError: the value might be an int
    assert callable(1 if _sv_verifier.nondet_bool() else f)
