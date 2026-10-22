# _sv_verifier.nondet_float_range(a, b) is random.uniform(a, b): a float if a
# and b are numbers, TypeError otherwise
import _sv_verifier
x = _sv_verifier.nondet_float_range(0, 1)
_sv_verifier.check_type(x, float)
y = _sv_verifier.nondet_float_range(True, 2.5)
_sv_verifier.check_type(y, float)
if _sv_verifier.nondet_bool():
    z = _sv_verifier.nondet_float_range("a", 1)
