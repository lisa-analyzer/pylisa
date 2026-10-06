class A:
    def __add__(self, o):
        return 1
    def __iadd__(self, o):
        return 2
    def __radd__(self, o):
        return 3

class B:
    def __sub__(self, o):
        return 10

class C:
    def __iadd__(self, o):
        return 2
    def __radd__(self, o):
        return 3

a = A()
r1 = a + 5
r2 = 5 + a
a += 5
r3 = a
b = B()
b -= 1
r4 = b
c = C()
c += 1
r5 = c
r6 = c + 1
d = C()
r10 = 1 + d
n = 7
n += 3
r7 = n
s = "ab"
s *= 2
r8 = s
f = 7.0
f //= 2
r9 = f
c1 = input()
if c1:
    t1 = 5
    t1 += "x"
if c1:
    t2 = 7
    t2 //= 0
after = 1
