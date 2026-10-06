import math
from os import path

class Calc:
    def double(self, k):
        return k * 2

    def add(self, a, b):
        return a + b

    def me(self):
        return self

c = Calc()
a = c.double(4)
b = c.add(2, 3)
d = Calc().double(5)
e = c.me().add(1, 1)
f = c.add(b=10, a=1)
r = math.floor(2.5)
p = path.join("x", "y")
