# copying references held by fields, tuples and lists keeps the objects
class K:
    def __init__(self):
        self.v = 1


class H:
    def __init__(self):
        self.f = K()


o = H()
y = o.f
a = y.v

t = (K(), 2)
w = t[0]
b = w.v

l = [K()]
x = l[0]
c = x.v

p, q = K(), K()
d = p.v


def f():
    i, j = K(), K()
    return i.v


e = f()
