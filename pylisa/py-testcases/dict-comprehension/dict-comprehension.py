import _sv_verifier

# the keys of a comprehension are not tracked: possible KeyError
pairs = [("a", 1), ("b", 2)]
d = {k: v + 1 for k, v in pairs}
x = d["a"] + 1
e = {i: str(i) for i in range(3) if i > 0}
y = e[1] + "s"

# no iterations: an empty dictionary
f = {k: 1 for k in []}
assert len(f) == 0

# unhashable keys: TypeError
if _sv_verifier.nondet_bool():
    g = {[1]: 2 for _ in range(2)}

# the keys of a dictionary are hashable: unpacking it raises no TypeError
h = {**{"z": 3}, **{k: v for k, v in pairs}}
d2 = {}
d2[pairs[0][0]] = 1
h2 = {**d2}
