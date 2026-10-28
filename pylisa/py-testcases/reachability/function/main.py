# the assertion failing in f is raised by the call f(), that is definitely
# reachable, but has no sub-expressions to read its reachability from: the
# violation is possible
import _sv_verifier


def f():
    assert 1 == 2


if _sv_verifier.nondet_bool():
    y = 1 // 0
f()
