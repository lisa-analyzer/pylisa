# module globals are visible inside functions
limit = 10


def read_global():
    return limit


r = read_global()


# a local that shadows a builtin does not affect the builtin elsewhere
def shadow():
    len = 3
    return len


s = shadow()
t = len("abc")


# functions held by module variables can be called through them
def double(x):
    return x * 2


g = double
u = g(4)


# a class read inside a function
class K:
    pass


def make():
    return K()


k = make()


# a function of the module referenced inside another function
def caller():
    return double(5)


v = caller()
