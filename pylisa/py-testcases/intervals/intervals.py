# the intervals of integers: comparisons refine the variables they compare,
# and the arithmetic follows python's integers (unbounded, // and % rounding
# towards minus infinity)
import _sv_verifier

a = _sv_verifier.nondet_int()
b = _sv_verifier.nondet_int()


def guarded(a, b):
    if not (1 <= b and b <= 1024):
        return 0
    # b is in [1, 1024]: no ZeroDivisionError
    q = a // b
    r = a % b
    assert 0 <= r and r <= 1023
    return q


guarded(a, b)

# x % n is in [0, n - 1] for a positive n, also for a negative x
i = a % 3
xs = [10, 20, 30]
v = xs[i]
assert v >= 10

# abs is not negative
n = abs(b)
assert n >= 0

# a loop counter: after the loop, the condition does not hold
c = 0
while c < 10:
    c = c + 1
assert c >= 10

# the branches refine the compared variable
if a > 5:
    assert a >= 6
else:
    assert a <= 5

# python's ints are unbounded: no wrap-around
big = 2147483647
big = big + 1
assert big > 2147483647

if _sv_verifier.nondet_bool():
    # surely fails: i is in [0, 2]
    assert i == 3
