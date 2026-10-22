# the exceptions raised in a try block (also by calls) are caught by the
# first except block whose classes they are instances of
import _sv_verifier
class A(Exception):
    def __init__(self, payload):
        self.payload = payload
class B(A):
    pass
raised = False
try:
    raise A(1)
except A:
    raised = True
assert raised
try:
    raise B(2)
except A as e:
    v = e.payload + 1
    assert v == 3
def f(x):
    if x:
        raise ValueError()
    return 1
caught = 0
try:
    f(True)
except ValueError:
    caught = 1
assert caught == 1
try:
    y = f(False)
except ValueError:
    y = 2
else:
    y = y + 10
assert y == 11
done = False
try:
    z = 1
finally:
    done = True
assert done
try:
    try:
        raise B(3)
    except ValueError:
        pass
except (A, TypeError) as e2:
    w = e2.payload
assert w == 3
k = 0
try:
    k = 1 + "a"
except TypeError:
    k = 5
assert k == 5

# an exception that no except block catches propagates, also when it is raised
# again by an except block
if _sv_verifier.nondet_bool():
    try:
        raise TypeError()
    except ValueError:
        pass
if _sv_verifier.nondet_bool():
    try:
        raise TypeError()
    except TypeError:
        raise
try:
    r = 1 + "a"
except TypeError:
    r = 0
assert r == 0
