a = b"abc"
b = a + b"de"
c = a * 2
d = 2 * a
e = a[0]
f = a[-1]
g = a[1:]
h = a[::-1]
i = len(a)
j = 97 in a
k = b"bc" in a
l = a == b"abc"
m = a == "abc"
n = a != "abc"
o = str(a)
p = repr(b"it's\n\x00\xff")
q = b"\x41\x42" + b'\101'
r = len(b"")
s = a * 0
t = a[10:]
u = "%s" % a
v = a[True]
c1 = input()
if c1:
    e1 = a + "x"
if c1:
    e2 = a[3]
if c1:
    e3 = 300 in a
if c1:
    e4 = "a" in a
if c1:
    e5 = a < "b"
if c1:
    e6 = a * 2.5
if c1:
    e7 = a["x"]
if c1:
    e8 = "x" + a
if c1:
    e9 = len(5)
after = 1
