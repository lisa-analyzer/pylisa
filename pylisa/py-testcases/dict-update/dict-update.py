# dict.update adds the entries of a dictionary (or of an iterable of pairs) and
# the keyword arguments, and returns None; the keys written after the creation
# of a dictionary are not tracked (see DictModel): reading them might raise
# KeyError
import _sv_verifier

d = {"a": 1}
d.update({"b": 2})
v = d["b"]
_sv_verifier.check_type(v, int)
d.update(c=3)
r = d.update({})
assert r is None
# TypeError: the elements of the iterable might not be pairs
d.update([("e", "s")])
if _sv_verifier.nondet_bool():
    # TypeError: not iterable
    d.update(5)
