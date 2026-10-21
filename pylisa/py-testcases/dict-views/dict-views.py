import _sv_verifier

d = {"a": 1, "b": 2}

# iterating the views: the keys, the values, and (key, value) tuples
for k, v in d.items():
    x = k + "!"
    y = v + 1
for k in d.keys():
    z = k + "?"
for v in d.values():
    w = v * 2
for t in d.items():
    u = t[1] + 1
assert all(isinstance(v, int) for v in d.values())
assert all(isinstance(k, str) for k, v in d.items())
ks = list(d.keys())
vs = list(d.values())

# the length of a view is the one of its dictionary, also after changes
assert len(d.keys()) == 2
assert len(d.items()) == 2
view = d.values()
n = _sv_verifier.nondet_dict(_sv_verifier.nondet_str, _sv_verifier.nondet_int)
m = len(n.keys()) + 1

# the values of the untracked keys
for k, v in n.items():
    r = v + 1

# the keys that are not tracked keep their types: reading with them raises
# no TypeError (only KeyError, as any key might be missing)
for k in n:
    a = k + "!"
    b = n[k] + 1
m2 = {**{"x": 1}, **n}
for k in m2:
    e = m2[k]
d2 = {}
d2[_sv_verifier.nondet_int()] = "v"
for k in d2:
    f = k + 1
o = {k: 1 for k in [1.5, 2.5]}
for k in o:
    g = k * 2
l = {}
l[_sv_verifier.nondet_str()] = 1
l |= n
for k in l:
    h = l[k]
