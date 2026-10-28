# an attribute is surely initialized on an object if it is assigned on all the
# paths reaching the read: otherwise, AttributeError might be raised
import _sv_verifier


class C:
    def __init__(self, v):
        if v:
            self.v = v
        self.w = 1

    def get(self):
        return self.v


c = C(_sv_verifier.nondet_int())
if _sv_verifier.nondet_bool():
    # AttributeError: v is not assigned when the int is 0
    x = c.v
if _sv_verifier.nondet_bool():
    # AttributeError, inside get
    x = c.get()
# assigned on all the paths
y = c.w


class D:
    pass


d = D()
if _sv_verifier.nondet_bool():
    d.a = 1
else:
    d.b = 2
if _sv_verifier.nondet_bool():
    # AttributeError: a is assigned only in a branch
    z = d.a
d.c = 3
t = d.c
