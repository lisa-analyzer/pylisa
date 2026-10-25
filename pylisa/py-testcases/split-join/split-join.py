# str.split yields a new list of strs (of unknown length), str.join a str
import _sv_verifier

s = _sv_verifier.nondet_str()
words = s.split()
n = len(words)
_sv_verifier.check_type(n, int)
for w in words:
    _sv_verifier.check_type(w, str)
parts = "a,b".split(",", 1)
_sv_verifier.check_type(parts, list)
if _sv_verifier.nondet_bool():
    # TypeError: the separator is not a str
    bad = s.split(1)
if _sv_verifier.nondet_bool():
    # ValueError: empty separator (not a property)
    bad = s.split("")
j = "-".join(words)
_sv_verifier.check_type(j, str)
k = "".join(["a", "b"])
_sv_verifier.check_type(k, str)
e = "".join([])
if _sv_verifier.nondet_bool():
    # TypeError: the elements are not strs
    bad = "".join([1, 2])
if _sv_verifier.nondet_bool():
    # TypeError: not iterable
    bad = "".join(5)
