import _sv_verifier


class A:
    def f(self):
        return 1


class B:
    pass


def g():
    return 1


# decided by the types
x = 3
assert isinstance(x, int)
assert not isinstance(x, str)
assert isinstance(True, int)
assert not isinstance(True, float)
assert isinstance(x, int | str)
assert isinstance(A(), A)
assert not isinstance(B(), A)

# the branches refine the types of the variable
y = "a" if _sv_verifier.nondet_bool() else 4
if isinstance(y, str):
    z = y + "b"
else:
    z = y + 1
if not isinstance(y, str):
    z = y + 1
o = A() if _sv_verifier.nondet_bool() else B()
if isinstance(o, A):
    o.f()
w = None if _sv_verifier.nondet_bool() else 2
if not isinstance(w, int | None):
    w.foo()

# a stored result does not refine the variable: possible TypeError
b = isinstance(y, int)
if b:
    q = y + 1

# not types: TypeError
if _sv_verifier.nondet_bool():
    isinstance(x, len)
if _sv_verifier.nondet_bool():
    isinstance(x, g)
if _sv_verifier.nondet_bool():
    isinstance(x, None)
if _sv_verifier.nondet_bool():
    isinstance(x, 3)
