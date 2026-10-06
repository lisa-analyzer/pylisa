from collections import OrderedDict

class V:
    def __add__(self, o):
        return 1

class W:
    def m(self):
        return 0

class Z(OrderedDict):
    def m(self):
        return 0

v = V()
w = W()
z = Z()
a = v + 1
b = w == 1
c = w != 1
d = 1 == w
e = w == w
c1 = input()
if c1:
    e1 = w + 1
if c1:
    e2 = 1 + w
if c1:
    e3 = v - 1
if c1:
    e4 = w < w
if c1:
    e5 = divmod(w, 2)
if c1:
    e6 = w ** 2
if c1:
    e7 = w + v
if c1:
    w2 = W()
    w2 += 1
if c1:
    u = z - 1
after = 1
