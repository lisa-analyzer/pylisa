class C:
    def m(self, x):
        return x + 1

    def n(self):
        return 1


c = C()
# through the class, the receiver is an explicit argument
b = C.m(c, 2)
assert b == 3
d = C.n(5)
assert d == 1
# TypeError: n expects self
e = C.n()
