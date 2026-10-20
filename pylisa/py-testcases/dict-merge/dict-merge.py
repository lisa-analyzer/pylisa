a = {"x": 1, "y": 2}
b = {"y": 3, "z": 4}
m = a | b
assert m["x"] == 1
assert m["y"] == 3
assert m["z"] == 4
assert len(m) == 3
assert a["y"] == 2
c = {"x": 1}
c |= {"x": 5}
assert c["x"] == 5
d = c
d |= {"x": 7}
assert c["x"] == 7

# TypeError: a list of pairs might be invalid; any key might then be missing
e = {"x": 1}
e |= [("y", 2)]

import _sv_verifier

n = _sv_verifier.nondet_dict(_sv_verifier.nondet_str, _sv_verifier.nondet_int)
# KeyError: the key might be missing
w = n["a"]

# TypeError: dict | int
bad = {"x": 1} | 1
