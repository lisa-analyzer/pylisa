# assigning an element (x[i] = v) of a value whose class has no __setitem__
# raises TypeError
import _sv_verifier


class A:
    pass


class B:
    def __setitem__(self, i, v):
        self.last = v


x = 5
if _sv_verifier.nondet_bool():
    x[0] = 1
s = "ab"
if _sv_verifier.nondet_bool():
    s[0] = "c"
n = None
if _sv_verifier.nondet_bool():
    n[0] = 3
a = A()
if _sv_verifier.nondet_bool():
    a[0] = 1
# the method of the class is called
b = B()
b[0] = 7
assert b.last == 7
# a list or an int: TypeError might be raised
y = 5 if _sv_verifier.nondet_bool() else [1, 2]
y[0] = 2
