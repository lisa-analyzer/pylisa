# functools.reduce(function, iterable[, initial]) calls function on the
# accumulated value and each element: the result is any of the values the
# accumulated value might have
from functools import reduce
import _sv_verifier


def mul(x, y):
    return x * y


a = reduce(mul, [1, 2, 3])
_sv_verifier.check_type(a, int)
# the list surely has an element: the result is not the initial value
b = reduce(mul, [1.5, 2.0], 1)
_sv_verifier.check_type(b, float)
c = reduce(lambda acc, s: acc + s, ["a", "b"], "")
_sv_verifier.check_type(c, str)
# a single element, with no initial value: function is not called
d = reduce(mul, [7])
assert d == 7
xs = _sv_verifier.nondet_list(_sv_verifier.nondet_int)
e = reduce(mul, xs, 1)
_sv_verifier.check_type(e, int)
if _sv_verifier.nondet_bool():
    # TypeError: xs might be empty, with no initial value
    reduce(mul, xs)
if _sv_verifier.nondet_bool():
    # TypeError: empty, with no initial value
    reduce(mul, [])
if _sv_verifier.nondet_bool():
    # TypeError: 5 is not callable
    reduce(5, [1, 2])
if _sv_verifier.nondet_bool():
    # TypeError: mul(1, "a")
    reduce(mul, [1, "a"], 1)
