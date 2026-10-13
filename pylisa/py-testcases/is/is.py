class P:
    pass

def greet(name):
    return "Hello, " + name

def other(x):
    return x

def make():
    def inner():
        return 1
    return inner

# a function defined once is a single object
say_hello = greet
q = len
a = greet == say_hello
b = greet != say_hello
c = greet is say_hello
d = greet is not say_hello
e = greet is other
f = q == len
g = q is len
h = q is str
# None is a singleton, and values of different classes are different objects
n = None
i = n is None
j = greet is None
k = 5 is None
l = 5 is "a"
p = P()
m = p is None
x = p is not None
# unknown: objects of the same class, ints, and functions defined
# by a def executed many times
o = p is p
t = 5 is 5
r = make() is make()
s = make() == make()
cnt = 0
while cnt < 2:
    def loop():
        return 2
    if cnt == 0:
        first = loop
    cnt = cnt + 1
same = first is loop
# the conditions select the branch
inp = input()
y = None if inp == "n" else 3
if y is None:
    w = 1
else:
    w = 2
z = None
if z is None:
    v = 1
else:
    v = 2
if p is not None:
    u = 1
else:
    u = 2
last = 1
