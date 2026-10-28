# the names that a module imports or assigns once, at its top level, are
# aliases of the members (or modules) they are bound to
import _sv_verifier
import pkg
from pkg import number, text

_sv_verifier.check_type(number(), int)
_sv_verifier.check_type(text(), str)
_sv_verifier.check_type(pkg.number(), int)
_sv_verifier.check_type(pkg.helpers.half(), float)
f = pkg.text
_sv_verifier.check_type(f(), str)
if _sv_verifier.nondet_bool():
    # code that is not known: maybe might be unbound
    pkg.maybe()
if _sv_verifier.nondet_bool():
    # code that is not known: twice is bound twice
    pkg.twice()
