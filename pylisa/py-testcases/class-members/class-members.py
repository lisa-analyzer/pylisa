# the attributes of classes read through the class (C.name)
import _sv_verifier


class K:
    v = 3
    w = 4


# a constant assigned in the body of the class
assert K.v == 3
# written through the class: not known
if _sv_verifier.nondet_bool():
    K.w = "s"
b = K.w
if _sv_verifier.nondet_bool():
    # b might be "s"
    c = b + 1
# all classes have the special attributes
n = K.__name__
if _sv_verifier.nondet_bool():
    # not defined by the class: AttributeError might be raised
    m = K.nope


class B:
    x = 0


class S(B):
    pass


# written through the subclass: S.x, also through its objects, is not the
# one of B
S.x = 1
assert B.x == 0
if _sv_verifier.nondet_bool():
    assert S.x == 0
if _sv_verifier.nondet_bool():
    assert S().x == 0
