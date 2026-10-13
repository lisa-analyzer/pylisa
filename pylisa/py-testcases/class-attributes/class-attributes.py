class K:
    v = 3
    neg = -1
    s = "k" + "s"
    n = None

    def m(self):
        return self.v

    def setv(self, x):
        self.v = x

class W:
    w = 1

class X:
    a, b = 1, 2
    t = 1
    t = 2

class E:
    pass

# assigned through the class: the value is unknown
W.w = 7

kk = K()
# read from the class, also inside methods
a = kk.v
c = kk.m()
d = kk.neg
e = kk.s
f = kk.n
# the attribute of the object hides the one of the class
kk.v = 8
g = kk.v
h = K().m()
o = K()
o.setv(9)
i = o.v
j = W().w
# not assigned by a single assignment: the value is unknown
x = X()
k = x.a
l = x.t
inp = input()
# AttributeError: 'E' object has no attribute 'v'
r = K() if inp == "r" else E()
if inp == "m":
    m = r.v
# TypeError: 'NoneType' object is not callable
if inp == "p":
    p = kk.n()
# limitation: the attribute is assigned on some paths only, and
# the class attribute (3) is not considered
q = K()
if inp == "q":
    q.v = 5
z = q.v
last = 1
