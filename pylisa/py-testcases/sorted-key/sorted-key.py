# sorted(iterable, key=f) calls f on the elements, and compares its results
import _sv_verifier

data = [{"city": "a", "n": 1}, {"city": "b", "n": 2}]
s1 = sorted(data, key=lambda x: x["city"])
for d in s1:
    _sv_verifier.check_type(d["n"], int)
s2 = sorted([3, 1], key=lambda v: -v)
for v in s2:
    _sv_verifier.check_type(v, int)
# the keys are all str: no TypeError
s3 = sorted([1, "a"], key=lambda v: str(v))
if _sv_verifier.nondet_bool():
    # TypeError: the keys are an int and a str
    sorted([1, "a"], key=lambda v: v)
if _sv_verifier.nondet_bool():
    # KeyError, raised by the key
    sorted(data, key=lambda x: x["zzz"])
