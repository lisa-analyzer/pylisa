import _sv_verifier
# the elements are unpacked into the targets
pairs = [(1, "a"), (2, "b")]
a = [k + 1 for k, v in pairs]
b = [v + "x" for k, v in pairs]
c = [x + y for (x, y) in [(1, 2)]]
# the length of a comprehension is not tracked
assert c == [3]
d = [h + 1 for h, *t in [[1, 2, 3]]]
e = [t for h, *t in [[1, 2, 3]]]
f = [n + 1 for (n) in [1]]
g = [i + j for i, (j, z) in [(1, (2, 3))]]
# ints cannot be unpacked: TypeError
if _sv_verifier.nondet_bool():
    bad = [k for k, v in [1, 2]]
# too many values: ValueError, not checked
if _sv_verifier.nondet_bool():
    bad2 = [k for k, v in [(1, 2, 3)]]
