def double(x):
    return x * 2

def inc(x):
    return x + 1

def apply(f, x):
    return f(x)

f = double
a = f(5)
f = inc
b = f(5)
c = apply(double, 7)
d = apply(inc, 7)
g = f
e = g(1)
c1 = input()
if c1:
    h = double
else:
    h = inc
k = h(10)
n = 5
if c1:
    e1 = n()
after = 1
