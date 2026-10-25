# zip(...) yields tuples with an element of each iterable, read when it is
# iterated
import _sv_verifier

names = ["a", "b"]
ages = [1, 2]
for name, age in zip(names, ages):
    _sv_verifier.check_type(name, str)
    _sv_verifier.check_type(age, int)
pairs = list(zip(names, ages))
for p in pairs:
    a, b = p
    _sv_verifier.check_type(a, str)
    _sv_verifier.check_type(b, int)
# lazy: the element appended after zip(...) is seen
xs = [1]
z = zip(xs)
xs.append("s")
for (x,) in z:
    if _sv_verifier.nondet_bool():
        # x might be "s": TypeError might be raised
        y = x + 1
if _sv_verifier.nondet_bool():
    # TypeError: not iterable
    bad = zip(names, 5)
for t in zip():
    # zip() has no elements: unreachable
    assert False
