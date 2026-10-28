# lists and tuples define no reflected operators (e.g. __radd__), nor the
# arithmetic and bitwise ones other than + and *: applying them to other
# values raises TypeError
import _sv_verifier

if _sv_verifier.nondet_bool():
    # TypeError
    a = [1] + (2,)
if _sv_verifier.nondet_bool():
    # TypeError
    b = 1 + [1]
if _sv_verifier.nondet_bool():
    # TypeError
    c = (1,) - (2,)
if _sv_verifier.nondet_bool():
    # TypeError
    d = -[1]
if _sv_verifier.nondet_bool():
    # TypeError
    e = None | (1,)
# no TypeError: tuples define +, lists define += (with any iterable), and
# comparisons, whose results are not tracked
f = (1,) + (2,)
xs = [1]
xs += (2,)
g = [1] < [2]
h = [1] == (1,)
