a = b"%d items" % 3
b = b"%s!" % b"hi"
c = b"%b" % b"x"
d = b"%5.2f|" % 2.5
e = b"%x" % 255
f = b"%r" % "é"
g = b"%a" % 5
h = b"%c" % 65
i = b"%c" % b"z"
j = b"%-5b|" % b"ab"
k = b"\xff%s" % b"\x00"
m = "ab" % b"ab"
n = "%s" % b"ab"
o = b"%r" % b"x"
c1 = input()
if c1:
    e1 = b"%s" % "x"
if c1:
    e2 = b"%s" % 5
if c1:
    e3 = b"ab" % b"x"
if c1:
    e4 = b"%c" % "a"
if c1:
    e5 = b"%q" % 5
if c1:
    e6 = b"%d" % b"x"
after = 1
