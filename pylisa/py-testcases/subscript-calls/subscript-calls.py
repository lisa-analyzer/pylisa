# the receiver and the index of a subscript, and the operands of in, are
# evaluated once
import _sv_verifier

n = _sv_verifier.nondet_int()
calls = [0]


def pair():
    return [n, 2 * n]


def counted():
    calls[0] = calls[0] + 1
    return [1, 2]


def index():
    calls[0] = calls[0] + 1
    return 0


# the list created by the first call is not referenced anymore: the one created
# by the second call has a known length
first = pair()[1]
second = pair()
assert len(second) == 2
x = second[1]

a = counted()[0]
assert calls[0] == 1
b = [1, 2][index()]
assert calls[0] == 2
c = 2 in counted()
assert calls[0] == 3
if _sv_verifier.nondet_bool():
    # surely fails
    assert calls[0] == 2
