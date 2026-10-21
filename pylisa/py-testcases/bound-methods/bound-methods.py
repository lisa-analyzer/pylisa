import _sv_verifier


class A:
    def __init__(self, n):
        self.n = n

    def get(self):
        return self.n

    def pick(self):
        return self.get


class B(A):
    def get(self):
        return self.n + 1

    def store(self):
        self.saved = self.get


# a method read without calling it is bound to the object
a = A(5)
f = a.get
assert f() == 5
assert a.pick()() == 5

# the method is the one of the class of the object
b = B(5)
g = b.get
assert g() == 6
b.store()
assert b.saved() == 6
assert B(1).pick()() == 2

# the arguments must be accepted: TypeError
if _sv_verifier.nondet_bool():
    f(1)


# an attribute holding an object yields the object
class P:
    def __init__(self, v):
        self.v = v


o = P(1)
o.x = P(2)
y = o.x
assert y.v == 2
pairs = [(1, 2)]
p = pairs[0]
assert p[1] == 2
