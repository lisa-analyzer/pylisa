# enumerate(...) yields tuples with an int and an element of the iterable,
# read when it is iterated
import _sv_verifier

names = ["a", "b"]
for i, name in enumerate(names):
    _sv_verifier.check_type(i, int)
    _sv_verifier.check_type(name, str)
pairs = list(enumerate(names, 1))
for p in pairs:
    j, n = p
    _sv_verifier.check_type(j, int)
    _sv_verifier.check_type(n, str)
# lazy: the element appended after enumerate(...) is seen
xs = [1]
e = enumerate(xs)
xs.append("s")
for k, x in e:
    if _sv_verifier.nondet_bool():
        # x might be "s": TypeError might be raised
        y = x + 1
if _sv_verifier.nondet_bool():
    # TypeError: not iterable
    bad = enumerate(5)
if _sv_verifier.nondet_bool():
    # TypeError: start is not an int
    bad = enumerate(names, "x")
