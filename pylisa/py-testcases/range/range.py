import _sv_verifier

n = _sv_verifier.nondet_int()

# a range is a value, whose elements are ints
r = range(3)
for i in r:
    a = i + 1
b = len(range(n)) + 1
c = list(range(2, n, 2))
d = [x + 1 for x in c]
assert isinstance(r, range)
assert type(r) is range
assert all(isinstance(x, int) for x in range(n))

# its length is not tracked: possible IndexError
e = list(range(n))[0]

# the arguments must be ints
if _sv_verifier.nondet_bool():
    range("a")
if _sv_verifier.nondet_bool():
    range(1, 2.5)
s = _sv_verifier.nondet_str() if _sv_verifier.nondet_bool() else 1
f = range(s)

# a zero step raises ValueError: the rest is not reached
range(1, 5, 0)
assert False
