# the hints bool, type (the classes), object (all the values), the exception
# classes and type(None), and the classes of the values given by type(...)
import _sv_verifier


class A(Exception):
    pass


class C:
    pass


_sv_verifier.check_type(True, bool)
_sv_verifier.check_type(bool(0), bool)
_sv_verifier.check_type(A(), Exception)
_sv_verifier.check_type(C, type)
_sv_verifier.check_type(int, type)
_sv_verifier.check_type(C(), object)
_sv_verifier.check_type(None, type(None))
_sv_verifier.check_type(object(), object)
assert type(True) is bool
assert type({}) is dict
assert type(zip()) is zip
assert type(None) is type(None)
assert isinstance(ValueError("x"), Exception)
assert bool(1) and not bool("") and not bool()
if _sv_verifier.nondet_bool():
    # TypeError: an int is not a bool
    _sv_verifier.check_type(1, bool)
if _sv_verifier.nondet_bool():
    # TypeError: len is a function
    _sv_verifier.check_type(len, type)
if _sv_verifier.nondet_bool():
    # TypeError: an object is not a class
    _sv_verifier.check_type(C(), type)
if _sv_verifier.nondet_bool():
    # TypeError: an int is not None
    _sv_verifier.check_type(1, type(None))
