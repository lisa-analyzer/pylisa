# assigning a slice of a list replaces its elements with the ones of an
# iterable: the list might then have other elements, at other indices
import _sv_verifier

xs = [1, 2, 3]
xs[0:2] = []
for x in xs:
    _sv_verifier.check_type(x, int)
xs[1:] = ["a"]
for y in xs:
    if _sv_verifier.nondet_bool():
        # y might be "a": TypeError might be raised
        y + 1
ys = [1, 2]
ys[:] = []
if _sv_verifier.nondet_bool():
    # ys might be empty: IndexError might be raised
    v = ys[0]
if _sv_verifier.nondet_bool():
    # TypeError: not iterable
    ys[0:1] = 5
if _sv_verifier.nondet_bool():
    # TypeError: the bounds are not ints
    ys["a":] = []
if _sv_verifier.nondet_bool():
    # ValueError (not a property): the step is 0
    ys[::0] = []
