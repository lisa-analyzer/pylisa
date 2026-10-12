class Point:
    def __init__(self, x):
        self.x = x
    def norm(self):
        return self.x

class Callable:
    def __call__(self, v):
        return v

class MyErr(Exception):
    pass

class Empty:
    pass

def twice(v):
    return v * 2

p = Point(3)
a = p.norm()
c = input()
# TypeError: 'int' object is not callable
if c == "b":
    b = p.x()
# AttributeError: 'Point' object has no attribute 'missing'
if c == "d":
    d = p.missing()
# functions stored in attributes are called without the receiver
p.f = twice
e = p.f(4)
# TypeError: 'str' object is not callable
p.s = "str"
if c == "g":
    g = p.s()
# TypeError: 'NoneType' object is not callable
p.n = None
if c == "h":
    h = p.n()
# TypeError: 'Point' object is not callable
p.o = Point(1)
if c == "i":
    i = p.o()
# objects with __call__ can be called (the result is unknown)
p.k = Callable()
k = p.k(2)
# library functions stored in attributes and variables (the result is unknown)
p.l = len
l = p.l("abc")
q = len
m = q("abc")
# classes with bases are not decided: no error
err = MyErr()
if c == "n":
    n = err.anything()
# either a method or a missing attribute: unknown, and the AttributeError
# comes from norm, reading self.x on the Empty object
r = Point(1) if c == "r" else Empty()
if c == "o":
    o = r.norm()
# the attribute is assigned before being called
t = 0
w = 0
while t < 3:
    if t > 0:
        w = p.u(t)
    p.u = twice
    t = t + 1
last = 1
