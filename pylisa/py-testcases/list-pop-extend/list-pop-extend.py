# list.pop removes an element and returns it, list.extend adds the elements of
# an iterable
import _sv_verifier

l = [1, 2, 3]
x = l.pop()
assert x == 3
# l might have lost any element: its length is not known anymore
y = l.pop(0)
m = [1, 2]
m.pop()
# after the first pop, m is not known to have an element: IndexError might be
# raised
m.pop()
if _sv_verifier.nondet_bool():
    # IndexError: m is empty
    z = m[1]
e = []
if _sv_verifier.nondet_bool():
    # IndexError: pop from an empty list
    w = e.pop()
n = [1]
n.extend(["a", "b"])
for v in n:
    if _sv_verifier.nondet_bool():
        # v might be "a": TypeError
        v + 1
if _sv_verifier.nondet_bool():
    # TypeError: not iterable
    n.extend(5)
# n surely has an element
last = n.pop()
