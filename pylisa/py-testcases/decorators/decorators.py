import _sv_verifier


def g2():
    return 2


def d2(h):
    return g2


def d1(h):
    return h() + 10


# the decorators are applied bottom-up: f = d1(d2(f)) = d1(g2) = 12
@d1
@d2
def f():
    return 0


assert f == 12


# a decorator replacing the function
def wrap(h):
    return g2


@wrap
def k(x):
    return x


assert k() == 2

# a decorator held by a variable
a = wrap


@a
def m():
    return 5


assert m() == 2


# a decorator returned by a call
def make(n):
    if n == 1:
        return wrap
    return d1


@make(1)
def p():
    return 7


assert p() == 2


@make(2)
def q():
    return 7


assert q == 17


# a decorator returning the function itself
def ident(h):
    return h


@ident
def r(x):
    return x + 1


assert r(1) == 2


# the value bound to a decorated function is not known inside other
# functions: their calls are code that is not known
@ident
def s(x):
    return 1 / x


def t():
    return s(0)


if _sv_verifier.nondet_bool():
    t()


# a decorator returning a value that might not be callable: TypeError
def maybe(h):
    if _sv_verifier.nondet_bool():
        return g2
    return 3


@maybe
def u():
    return 0


y = u()
assert y == 2
