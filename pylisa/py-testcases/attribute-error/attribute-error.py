class Point:
    kind = "point"

    def __init__(self, x):
        self.x = x

    def norm(self):
        return self.x

    def get_y(self):
        return self.y

class Empty:
    pass

class MyErr(Exception):
    pass

p = Point(3)
a = p.x
b = p.kind
m = p.norm
n = p.norm()
c = input()
if c == "y":
    d = p.y
if c == "get":
    e = p.get_y()
p.y = 4
f = p.y
g = p.get_y()
q = Point(1)
if c == "q":
    h = q.y
o = Empty()
if c == "o":
    i = o.z
err = MyErr()
j = err.anything if c == "err" else 0
cls = p.__class__
r = Empty() if c == "r" else p
k = r.x if c == "k" else 0
w = Empty()
t = 0
s = 0
while t < 3:
    if t > 0:
        s = w.v
    w.v = t
    t = t + 1
last = s
