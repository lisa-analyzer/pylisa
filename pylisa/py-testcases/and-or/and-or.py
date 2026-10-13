class P:
    def __init__(self):
        self.v = 4

def boom():
    return 1 / 0

def five():
    return 5

# the result is one of the operands
a1 = True and False
a2 = 0 and 5
a3 = 3 and "x"
a4 = "" or "d"
a5 = None or 0
a11 = 1 and 2 and 3
a12 = 0 or "" or None
a13 = 1 < 2 and 2 < 3
a10 = five() or 0
# the right operand is not evaluated when the left one decides the result
a6 = 1 or boom()
a7 = 0 and boom()
x = None
a8 = x == None or x.v
n = 0
a16 = n != 0 and 10 / n
# objects are true
p = P()
a9 = p and p.v
c = input()
a14 = c or "default"
a15 = c and 1
# the conditions select the branch
if 0 or "":
    w = 1
else:
    w = 2
if 1 and "a":
    v = 1
else:
    v = 2
# ZeroDivisionError, if the input is not empty
if c and boom():
    u = 1
last = 1
