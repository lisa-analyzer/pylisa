# objects whose class defines __call__ can be called
class Adder:
    def __init__(self, k):
        self.k = k

    def __call__(self, x):
        return x + self.k


add2 = Adder(2)
a = add2(3)
fs = [add2]
b = fs[0](10)


# classes held by variables create objects when called
class Point:
    def __init__(self, x):
        self.x = x


P = Point
p = P(4)
c = p.x


def make(cls, v):
    return cls(v)


q = make(Point, 6)
d = q.x


# objects whose class does not define __call__ cannot be called
class Plain:
    pass


objs = [Plain()]
if input():
    objs[0]()
last = 1
