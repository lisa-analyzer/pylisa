class Counter:
    def __init__(self):
        self.n = 0

    def get(self):
        return self.n

def tick(c):
    c.n = c.n + 1
    return c.n

cnt = Counter()
r0 = cnt.n
cnt.n = 5
r1 = cnt.n
x = tick(cnt)
r2 = cnt.n
g = cnt.get()
cnt.s = "a"
r3 = cnt.s + "b"
other = Counter()
r4 = other.n
alias = cnt
alias.n = 10
r5 = cnt.n
cnt.n += 1
r6 = cnt.n
cnt.o = other
r7 = cnt.o.n
cnt.o.n = 3
r8 = other.n
c = input()
if c == "x":
    cnt.k = 1
r9 = cnt.k if c == "x" else 0

class K:
    v = 3

    def m(self):
        return 4

kk = K()
r10 = kk.v
kk.v = 8
r11 = kk.v
meth = kk.m
r12 = meth()
s = "abc"
up = s.upper
r13 = up()
i = 0
last = Counter()
while i < 3:
    last = Counter()
    last.n = i
    i = i + 1
r14 = last.n
if c == "attr":
    bad = cnt.missing
after = 1
