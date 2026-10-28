# a method of a class is only called on the instances of the class: in
# c.func(), C.func is called on the Cs, and D.func on the Ds
import _sv_verifier


class A:
    def func(self):
        return "a"


class B(A):
    def func(self):
        return super().func()


class C(B):
    def func(self):
        return super().func()


class D(B):
    def func(self):
        return len(super().func())

    def only_d(self):
        return 1


class E(B):
    def func(self):
        # TypeError: the Es are not Cs
        return len(super(C, self).func())


if _sv_verifier.nondet_bool():
    c = C()
else:
    c = D()
# no TypeError: D.func is not called on the Cs
d = c.func()

if _sv_verifier.nondet_bool():
    e = C()
else:
    e = E()
if _sv_verifier.nondet_bool():
    # TypeError, from E.func
    e.func()
