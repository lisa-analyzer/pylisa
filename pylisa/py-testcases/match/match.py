def f(v):
    match v:
        case "a":
            return 1
        case 2 | 3:
            return 2
        case None:
            return 3
        case -1:
            return 4
        case x if x == 10:
            return x + 1
        case _:
            return 0

class Color:
    RED = 1

assert f("a") == 1
assert f(3) == 2
assert f(None) == 3
assert f(-1) == 4
assert f(10) == 11


def h(v):
    match v:
        case 0:
            # ZeroDivisionError
            return 1 // v
        case _:
            return 2


x = h(0)
