# map(function, *iterables) calls function on an element of each iterable when
# it is iterated; unpacking it calls function on the elements in order
import _sv_verifier


def ident(x):
    return x


def inc(x):
    return x + 1


i = _sv_verifier.nondet_int()
s = _sv_verifier.nondet_str()
x, y, z = map(ident, [i, s, 3.0])
_sv_verifier.check_type(x, int)
_sv_verifier.check_type(y, str)
_sv_verifier.check_type(z, float)
for v in map(inc, [1, 2]):
    _sv_verifier.check_type(v, int)
ws = list(map(str, [1, 2]))
for w in ws:
    _sv_verifier.check_type(w, str)
us = [u for u in map(lambda a, b: a * b, [1, 2], [3, 4])]
for u in us:
    _sv_verifier.check_type(u, int)
# lazy: the element appended after map(...) is seen
xs = [1]
m = map(inc, xs)
xs.append("a")
if _sv_verifier.nondet_bool():
    # TypeError: inc("a")
    for t in m:
        pass
if _sv_verifier.nondet_bool():
    # TypeError: not iterable
    map(inc, 5)
if _sv_verifier.nondet_bool():
    # TypeError: no iterables
    map(inc)
