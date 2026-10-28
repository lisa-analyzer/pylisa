# the classes of collections.namedtuple create tuples whose elements are also
# their attributes, named by the fields
import _sv_verifier
from collections import namedtuple

Point = namedtuple("Point", ["x", "y"])
p = Point(1, "s")
_sv_verifier.check_type(p, tuple)
_sv_verifier.check_type(p.x, int)
_sv_verifier.check_type(p.y, str)
_sv_verifier.check_type(p[1], str)
assert len(p) == 2
q = Point(y=2, x=3.0)
_sv_verifier.check_type(q.x, float)
a, b = p
_sv_verifier.check_type(a, int)
Pair = namedtuple("Pair", "first, second")
r = Pair(1, 2)
_sv_verifier.check_type(r.second, int)
if _sv_verifier.nondet_bool():
    # TypeError: too few arguments
    Point(1)
if _sv_verifier.nondet_bool():
    # TypeError: no field z
    Point(1, 2, z=3)
if _sv_verifier.nondet_bool():
    # IndexError
    p[2]
if _sv_verifier.nondet_bool():
    # AttributeError: the attributes of a namedtuple cannot be assigned
    p.x = 2
