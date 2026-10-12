class Plain:
    pass

class MyErr(Exception):
    pass

def check(x):
    if x < 0:
        raise ValueError("negative")
    return x * 2

def by_class():
    raise IndexError

def mine():
    raise MyErr()

def mine_class():
    raise MyErr

def not_exception():
    raise 5

def plain():
    raise Plain()

def chained():
    raise TypeError("t") from ValueError("v")

def stored():
    err = ValueError("v")
    raise err

a = check(3)
b = check(-1) if input() == "b" else 0
if input() == "c":
    c = by_class()
if input() == "d":
    d = mine()
if input() == "e":
    e = mine_class()
if input() == "f":
    f = not_exception()
if input() == "g":
    g = plain()
if input() == "h":
    h = chained()
if input() == "i":
    i = stored()
z = 1
if input() == "q":
    raise ValueError("bye")
last = 2
