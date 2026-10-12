class Plain:
    pass

class Two:
    def __init__(self, x, y=2):
        self.s = x + y

class Bad:
    def __init__(self):
        return 5

class Maybe:
    def __init__(self, c):
        if c == "r":
            return 1
        return None

class NoneRet:
    def __init__(self):
        self.v = 1
        return None

class WithNew:
    def __new__(cls, x):
        return object.__new__(cls)

p0 = Plain()
t1 = Two(1)
r1 = t1.s
t2 = Two(1, 5)
r2 = t2.s
t3 = Two(y=3, x=4)
r3 = t3.s
n = NoneRet()
w = WithNew(1)
r4 = n.v
c = input()
if c == "a":
    p = Plain(1)
if c == "b":
    t = Two()
if c == "c":
    t4 = Two(1, 2, 3)
if c == "d":
    t5 = Two(z=1)
if c == "e":
    b = Bad()
m = Maybe(c)
after = 1
