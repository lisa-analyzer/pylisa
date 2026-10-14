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

i = _sv_verifier.nondet_int()
f = _sv_verifier.nondet_float()
s = _sv_verifier.nondet_str()
n = i + 1
k1 = _sv_verifier.check_type(i, int)
k2 = _sv_verifier.check_type(True, int)
k3 = _sv_verifier.check_type(f, float)
k4 = _sv_verifier.check_type(s, str)
k5 = _sv_verifier.check_type(None, None)
k6 = _sv_verifier.check_type(b"x", bytes)
if _sv_verifier.nondet_bool():
    k7 = _sv_verifier.check_type(s, len)
if _sv_verifier.nondet_bool():
    k8 = _sv_verifier.check_type(s, other_func)
k9 = _sv_verifier.check_type(1, float)
