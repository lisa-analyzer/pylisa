import _sv_verifier

s = "abc"
if _sv_verifier.nondet_bool():
    s = 3
r = s + "d"
