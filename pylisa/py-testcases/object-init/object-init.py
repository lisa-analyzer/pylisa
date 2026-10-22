# a class that does not define __init__ is initialized by object.__init__,
# that does nothing, returns None, and accepts no arguments
import _sv_verifier
class A:
    def f(self):
        return 1
class B(A):
    pass
a = A()
_sv_verifier.check_type(a, A)
b = B()
_sv_verifier.check_type(b, B)
assert b.f() == 1
if _sv_verifier.nondet_bool():
    c = A(1)
