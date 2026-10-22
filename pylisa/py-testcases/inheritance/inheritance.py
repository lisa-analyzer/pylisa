import _sv_verifier


# the method resolution order: C(A, B) uses A.func, D(B, A) uses B.func
class A:
    def func(self):
        return 42


class B:
    def func(self):
        return "b"


class C(A, B):
    pass


class D(B, A):
    pass


assert C().func() == 42
assert D().func() == "b"
assert isinstance(C(), B)


# a diamond: E(F, G), with F(Top) and G(Top) overriding func
class Top:
    def func(self):
        return "top"


class F(Top):
    pass


class G(Top):
    def func(self):
        return 1


class E(F, G):
    pass


assert E().func() == 1
assert F().func() == "top"


# super() continues in the method resolution order of the object
class S1:
    def name(self):
        return "s1"


class S2(S1):
    def name(self):
        return "s2" + super().name()


class S3(S2):
    def name(self):
        return "s3" + super().name()


assert S3().name() == "s3s2s1"


# __init__ and static methods are inherited
class Base:
    def __init__(self, v):
        self.v = v

    @staticmethod
    def twice(x):
        return x * 2


class Child(Base):
    def get(self):
        return self.v


assert Child(3).get() == 3
assert Child(3).twice(4) == 8
assert Child.twice(5) == 10


# a subclass overriding __init__ without calling super().__init__: the
# attribute set by the base is missing
class NoInit(Base):
    def __init__(self):
        self.w = 0


o = NoInit() if _sv_verifier.nondet_bool() else Child(1)
if isinstance(o, Child):
    # the branches also refine the objects o points to
    assert o.get() == 1
x = o.v
