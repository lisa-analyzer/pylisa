# dict(iterable, **kwargs) is a new dictionary with the entries of iterable (a
# mapping, or an iterable of key-value pairs), and then the keyword arguments
import _sv_verifier

a = dict()
_sv_verifier.check_type(a, dict)
if _sv_verifier.nondet_bool():
    # KeyError: a is empty
    a["x"]
b = dict(x=1, y="s")
_sv_verifier.check_type(b["x"], int)
_sv_verifier.check_type(b["y"], str)
# a copy
e = dict(b)
_sv_verifier.check_type(e["x"], int)
# the pairs of zip, whose number is not known
keys = ["a", "b"]
values = [1, 2]
c = dict(zip(keys, values))
for k in c:
    _sv_verifier.check_type(k, str)
d = dict([("p", 1.5)], q=2)
# KeyError might be raised (a loss of precision): the keys of the pairs are
# not tracked, and the dictionaries with untracked keys might have lost any
# key (see DictModel.read)
_sv_verifier.check_type(d["q"], int)
for v in d.values():
    if _sv_verifier.nondet_bool():
        # TypeError: v might be a float
        v[0]
if _sv_verifier.nondet_bool():
    # TypeError: not iterable
    dict(5)
