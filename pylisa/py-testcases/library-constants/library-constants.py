# the constants of the modules of the standard library
import _sv_verifier
import math
import math as m
import re

p = math.pi
_sv_verifier.check_type(p, float)
assert p > 3.14 and p < 3.15
assert m.pi == math.pi
flags = re.IGNORECASE | re.UNICODE
_sv_verifier.check_type(flags, int)
assert flags == 34
if _sv_verifier.nondet_bool():
    # not modeled: its value is not known
    e = math.e
