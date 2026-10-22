# the values of the builtin classes have only the attributes of their class:
# reading another one raises AttributeError
import _sv_verifier
x = 5
if _sv_verifier.nondet_bool():
    x.strip()
y = None
if _sv_verifier.nondet_bool():
    y.foo()
z = "a"
if _sv_verifier.nondet_bool():
    z.nothere()
w = 2.5
if _sv_verifier.nondet_bool():
    w.append(1)
ok1 = x.bit_length()
ok2 = z.upper()
ok3 = w.is_integer()
r = x.real
if _sv_verifier.nondet_bool():
    q = x.nope
t = True
ok4 = t.bit_count()
# a value that is not known (here, the result of a call that is not resolved)
# might be of any class: no error is raised
class K:
    def m(self):
        return 1
k = K
u = k()
ok5 = u.m()
ok6 = u.m
