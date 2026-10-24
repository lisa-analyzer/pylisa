# a violation is definite when the statement is definitely reachable, and
# every execution reaching it raises the exception there
import _sv_verifier

if _sv_verifier.nondet_bool():
    # possibly reachable: possible
    y = 1 + "a"
n = _sv_verifier.nondet_int()
for i in range(3):
    n = n + 1
q = 0
if n > 0:
    z = 1
else:
    z = 2
q = 1
if _sv_verifier.nondet_bool():
    # z is 1 or 2, not known by constant propagation: possible
    assert z == 3
q = 2
# reachable after the branches and the loop, and surely raising: definite
w = {"x": 1} | 1
