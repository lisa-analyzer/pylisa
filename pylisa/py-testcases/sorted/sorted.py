# sorted(...) yields a new list with the elements of the iterable
import _sv_verifier

xs = [3, 1, 2]
ys = sorted(xs)
for y in ys:
    _sv_verifier.check_type(y, int)
zs = sorted(["b", "a"], reverse=True)
for z in zs:
    _sv_verifier.check_type(z, str)
e = sorted([])
if _sv_verifier.nondet_bool():
    # TypeError: an int and a str cannot be compared
    bad = sorted([1, "a"])
if _sv_verifier.nondet_bool():
    # TypeError: not iterable
    bad = sorted(5)
if _sv_verifier.nondet_bool():
    # key is called on the elements (ints): it raises nothing
    k = sorted(xs, key=lambda v: -v)
