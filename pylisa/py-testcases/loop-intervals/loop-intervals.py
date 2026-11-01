# the elements of ranges are between their bounds, products in loops do not
# lose their sign, and the condition of an assert holds after it
import _sv_verifier

xs = [0, 1, 2, 3, 4, 5, 6, 7]
total = 0
for n in range(1, 9):
    total = total + xs[-n]
for i in range(8):
    total = total + xs[i]
for j in range(7, -1, -1):
    total = total + xs[j]
r = 1
k = _sv_verifier.nondet_int()
while k > 0 and r < 1000:
    r = r * 2
    k = k - 1
q = 10 // r
assert r >= 1
for e in range(0):
    assert False
if _sv_verifier.nondet_bool():
    # surely fails: i is at most 7
    for i in range(8):
        assert i < 7

m = _sv_verifier.nondet_int()
assert 1 <= m and m <= 8
y = xs[-m]

# an empty range has no element: the comprehension is an empty list
b = [e for e in range(0)]
_sv_verifier.check_type(b, list)
