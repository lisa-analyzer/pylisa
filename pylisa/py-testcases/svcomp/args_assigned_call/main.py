import _sv_verifier


def param_func():
    return "Hello from param_func"


def other_func():
    return 42


def func(a):
    return a()


if _sv_verifier.nondet_bool():
    b = param_func
else:
    b = other_func
c = func(b)

if b is param_func:
    _sv_verifier.check_type(c, str)
    assert c == "Hello from param_func"
else:
    _sv_verifier.check_type(c, int)
    assert c == 42
