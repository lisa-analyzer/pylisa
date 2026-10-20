a = {"x": 1, "y": 2}
b = {"y": 3, "z": 4}
m = {**a, **b}
assert m["x"] == 1
assert m["y"] == 3
assert m["z"] == 4
assert len(m) == 3
n = {**a, "x": 9}
assert n["x"] == 9
o = {"x": 0, **a}
assert o["x"] == 1
e = {**{}}
assert len(e) == 0

# TypeError: a list is not a mapping
bad = {**[1, 2]}
