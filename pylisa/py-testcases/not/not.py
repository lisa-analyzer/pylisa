class P:
    pass

class L:
    def __len__(self):
        return 0

def f():
    return 1

# not is defined for values of any type, on their truthiness
n1 = not True
n2 = not 5
n3 = not ""
n4 = not None
n5 = not "a"
n6 = not input()
n7 = not 0.0
n8 = not b""
# objects without __bool__ and __len__, and functions, are true
n9 = not P()
n10 = not f
# __len__ is not called: the result is unknown
n11 = not L()
n12 = not (3 < 2)
n13 = 2 not in (1, 2)
# not in conditions selects the branch
x = 0
if not x:
    w = 1
else:
    w = 2
y = "abc"
if not y:
    v = 1
else:
    v = 2
c = input()
if not c:
    u = 1
else:
    u = 2
last = 1
