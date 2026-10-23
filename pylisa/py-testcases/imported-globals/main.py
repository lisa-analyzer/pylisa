# calling functions of an imported module that read the globals of that module
# raises no NameError: the globals are bound when the module is imported
import _sv_verifier
from helper import get, k_value

x = get()
_sv_verifier.check_type(x, str)
# x is "a": the assertion surely fails
if _sv_verifier.nondet_bool():
    assert x == "b"
# the value of K is not known here
y = k_value()
