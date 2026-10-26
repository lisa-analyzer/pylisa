# the package re-exports a function of one of its modules, aliases a function
# of another one, and imports a module
import _sv_verifier
from .mod import number
from . import other
text = other.text
from . import helpers
if _sv_verifier.nondet_bool():
    # bound conditionally: not an alias
    from .mod import maybe
# bound twice: not an alias
from .mod import twice
twice = other.text
