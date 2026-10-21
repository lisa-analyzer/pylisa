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


# the methods of builtin values, and of library classes
up = "abc".upper
assert up() == "ABC"
lst = [3, 1]
add = lst.append
add(5)


# the methods read from a class are plain functions
class Q:
    def __init__(self, n):
        self.n = n

    def get(self):
        return self.n

    @staticmethod
    def twice(v):
        return v * 2


class R(Q):
    pass


fn = Q.get
assert fn(Q(4)) == 4
tw = R.twice
assert tw(5) == 10
assert R.get(R(7)) == 7
if _sv_verifier.nondet_bool():
    fn()
