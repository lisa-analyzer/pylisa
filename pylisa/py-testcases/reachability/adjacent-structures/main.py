# the condition of the if is also the end of the loop: the reachability is
# not restored after the loop (see Reachability), so the violation is possible
import _sv_verifier

if _sv_verifier.nondet_bool():
    y = 1 // 0
n = _sv_verifier.nondet_int()
for i in range(2):
    n = n + 1
if _sv_verifier.nondet_bool():
    n = 2
u = {"x": 1} | 2
