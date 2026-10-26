# the concatenation of two lists is a new list with the elements of the first,
# and then the ones of the second
import _sv_verifier

a = [1, "x"] + [2.0]
_sv_verifier.check_type(a[0], int)
_sv_verifier.check_type(a[1], str)
_sv_verifier.check_type(a[2], float)
if _sv_verifier.nondet_bool():
    # IndexError: a has 3 elements
    a[3]
# the elements of the second list are at unknown indices, as its length is
# not known
b = [1] + _sv_verifier.nondet_list(_sv_verifier.nondet_int)
_sv_verifier.check_type(b[0], int)
for x in b:
    _sv_verifier.check_type(x, int)
# the length of the first list is not known: all the elements are at unknown
# indices, and the list surely has an element
c = _sv_verifier.nondet_list(_sv_verifier.nondet_str) + [1]
d = c[0]
for y in c:
    if _sv_verifier.nondet_bool():
        # TypeError: y might be a str
        y + 1
# the operands are not changed
_sv_verifier.check_type(a[0], int)
e = [] + []
if _sv_verifier.nondet_bool():
    # IndexError: e is empty
    e[0]
