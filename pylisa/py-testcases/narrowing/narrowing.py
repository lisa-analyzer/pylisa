class P:
    def __init__(self):
        self.v = 4

first = P()
inp = input()
# x is None or an object
x = None if inp == "n" else P()
# the branches of x is None refine the type (and the value) of x
if x is None:
    a = x
    c = 0
else:
    a = x
    c = x.v
if x is not None:
    d = x.v
else:
    d = x
e = 0 if x is None else x.v
f = x is not None and x.v
y = None if inp == "n" else 3
if y is None:
    g = y
else:
    g = y + 1
# also on attributes
first.o = None if inp == "o" else 7
if first.o is None:
    h = first.o
else:
    h = first.o + 1
z = None
if z is None:
    k = 1
else:
    k = 2
# an object is true: both branches are possible
if x:
    m = 1
else:
    m = 2
last = 1
