class C:
    def __init__(self):
        self.n = 0

class L:
    def __len__(self):
        return 7

def bump(c):
    c.n = c.n + 1
    return (1, 2)

def sbump(c):
    c.n = c.n + 1
    return "ab"

# the argument is evaluated once
c = C()
k = len(bump(c))
n = c.n
d = C()
s = len(sbump(d))
m = d.n
e = C()
q = len
r = q(bump(e))
o = e.n
t = len((1, 2, 3))
# __len__ of a class of the program
l = len(L())
inp = input()
# TypeError: object of type 'int' has no len()
if inp == "i":
    i = len(5)
last = 1
