import _sv_verifier


class A:
    pass


# lists and tuples of known length: element by element
assert sum([1, 2]) == 3
assert sum((1, 2.5)) == 3.5
assert sum([], 7) == 7
assert sum([1, 2], 10) == 13
assert all([1, True, "a"])
assert not all([1, 0])
assert any([0, "", 3])
assert not any([0, None, ""])
assert all([])
assert not any([])
e = _sv_verifier.nondet_int()
assert any([e, True])
assert all([e, True]) == (e != 0)

# any of the elements
extra = _sv_verifier.nondet_list(_sv_verifier.nondet_int)
d = sum(extra) + 1
f = sum(range(e)) + 1
g = sum(x * 2 for x in extra) + 1
assert all(isinstance(i, int) for i in extra)
assert not any(isinstance(i, str) for i in extra)
assert all(i == 1 for i in extra)

# TypeError
if _sv_verifier.nondet_bool():
    sum([1, "a"])
if _sv_verifier.nondet_bool():
    strs = _sv_verifier.nondet_list(_sv_verifier.nondet_str)
    sum(strs)
if _sv_verifier.nondet_bool():
    all(3)

# objects whose class does not define __radd__ cannot be added
if _sv_verifier.nondet_bool():
    sum([A()])


class B:
    def __radd__(self, other):
        return other


# the additions of objects whose class defines __radd__ are not known
h = sum([B()])
