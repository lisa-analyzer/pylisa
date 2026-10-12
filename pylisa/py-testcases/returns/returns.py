def sign(x):
    if x > 0:
        return 1
    if x < 0:
        return -1

def find(s, ch):
    i = 0
    while i < len(s):
        if s[i] == ch:
            return i
        i = i + 1

def early(x):
    if x:
        return
    return 5

def nothing():
    x = 1

def bare():
    return

a = sign(5)
b = sign(-3)
c = sign(0)
d = find("abc", "c")
e = find("abc", "z")
f = early(True)
g = early(False)
h = nothing()
i = bare()
j = c == None
k = h == None
