# sets are objects whose elements have no order, and whose number is not
# tracked
import _sv_verifier

s = {1, 2}
_sv_verifier.check_type(s, set)
assert type(s) is set
for x in s:
    _sv_verifier.check_type(x, int)
t = set([1, "a"])
for y in t:
    if _sv_verifier.nondet_bool():
        # TypeError: y might be "a"
        y + 1
for z in set():
    # unreachable: no elements
    assert False
s.add("b")
for w in s:
    if _sv_verifier.nondet_bool():
        # TypeError: w might be "b"
        w + 1
v = {1}
# not modeled: the elements are not known afterwards
v.update(["c"])
for q in v:
    if _sv_verifier.nondet_bool():
        # TypeError: q is not known
        q + 1
_sv_verifier.check_type(len(s), int)
if _sv_verifier.nondet_bool():
    # TypeError: not iterable
    set(5)
if _sv_verifier.nondet_bool():
    # KeyError: 3 might not be an element
    s.remove(3)
s.discard(5)
# KeyError might be raised: the number of elements is not tracked
k = s.pop() if _sv_verifier.nondet_bool() else 0
c = s.copy()
for e in c:
    if _sv_verifier.nondet_bool():
        # TypeError: e might be "b"
        e + 1
s.update(["a"], (2.5,))
if _sv_verifier.nondet_bool():
    # TypeError: not iterable
    s.update(5)
if _sv_verifier.nondet_bool():
    # AttributeError: the attributes of a set cannot be assigned
    s.x = 1
