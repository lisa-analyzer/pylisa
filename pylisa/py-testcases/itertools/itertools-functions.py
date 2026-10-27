# the iterators of itertools are lazy: their elements are computed when they
# are iterated, from the elements of the iterables they hold
import itertools
import _sv_verifier

c = itertools.count(5)
_sv_verifier.check_type(next(c), int)
f = itertools.count(1.5, 2)
_sv_verifier.check_type(next(f), float)
for ch in itertools.cycle("AB"):
    _sv_verifier.check_type(ch, str)
    break
for h in itertools.repeat("h", 3):
    _sv_verifier.check_type(h, str)
for x in itertools.chain([1, 2], (3,)):
    _sv_verifier.check_type(x, int)
for a, b in itertools.permutations([1, 2, 3], 2):
    _sv_verifier.check_type(a, int)
for p in itertools.product("AB", [1]):
    _sv_verifier.check_type(p[0], str)
    _sv_verifier.check_type(p[1], int)
for k, g in itertools.groupby([{"c": 1}, {"c": 2}], key=lambda d: d["c"]):
    _sv_verifier.check_type(k, int)
    for d in g:
        _sv_verifier.check_type(d["c"], int)
for e in itertools.compress([1, 2], [True, False]):
    _sv_verifier.check_type(e, int)
# a default instead of StopIteration
n = next(itertools.chain(), 0)
if _sv_verifier.nondet_bool():
    # TypeError: a list is not an iterator
    next([1])
if _sv_verifier.nondet_bool():
    # TypeError: not iterable
    itertools.cycle(5)
if _sv_verifier.nondet_bool():
    # TypeError: r is not an int
    itertools.combinations([1], "a")
if _sv_verifier.nondet_bool():
    # KeyError, raised by the key
    for k2, g2 in itertools.groupby([{"c": 1}], key=lambda d: d["z"]):
        pass
