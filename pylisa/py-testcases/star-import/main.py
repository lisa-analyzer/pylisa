# from helper import * binds the names of helper that do not start with an
# underscore
import _sv_verifier
from helper import *

a = one()
assert a == 1
b = text()
_sv_verifier.check_type(b, str)
if _sv_verifier.nondet_bool():
    # max is the one of helper, whose value is not known here (it is not the
    # builtin max): the call is code that is not known
    m = max(a, 2)
