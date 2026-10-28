# tuple(iterable) is a new tuple with the elements of iterable, or the
# iterable itself when it is a tuple
import _sv_verifier

a = (1, 2)
_sv_verifier.check_type(a, tuple)
assert type(a) is tuple
b = tuple([3, "s"])
_sv_verifier.check_type(b, tuple)
_sv_verifier.check_type(b[0], int)
_sv_verifier.check_type(b[1], str)
assert len(b) == 2
e = tuple()
assert len(e) == 0
xs = _sv_verifier.nondet_list(_sv_verifier.nondet_int)
f = tuple(xs)
_sv_verifier.check_type(f, tuple)
if _sv_verifier.nondet_bool():
    # IndexError: f might be empty
    f[0]
if _sv_verifier.nondet_bool():
    # TypeError: not iterable
    tuple(5)
if _sv_verifier.nondet_bool():
    # TypeError: a list is not a tuple
    _sv_verifier.check_type([1], tuple)
