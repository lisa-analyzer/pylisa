# a single element followed by a comma is a tuple, also between parentheses,
# while a parenthesized expression is not
import _sv_verifier

t = (1,)
u = 2,
g = (5)
_sv_verifier.check_type(g, int)
if _sv_verifier.nondet_bool():
    # TypeError: a tuple is not an int
    _sv_verifier.check_type(t, int)
if _sv_verifier.nondet_bool():
    # TypeError: a tuple is not an int
    _sv_verifier.check_type(u, int)
a, = [3]
_sv_verifier.check_type(a, int)
(b,) = (4,)
_sv_verifier.check_type(b, int)
for z, in [(1,), (2,)]:
    _sv_verifier.check_type(z, int)
ws = [y for y, in [(1,), (2,)]]
for w in ws:
    _sv_verifier.check_type(w, int)
