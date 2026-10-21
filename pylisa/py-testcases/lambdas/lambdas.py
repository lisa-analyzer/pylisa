inc = lambda x: x + 1
assert inc(1) == 2
add = lambda x, y=10: x + y
assert add(1) == 11
assert add(1, 2) == 3
first = lambda *xs: len(xs)
assert first(1, 2, 3) == 3
kw = lambda a, *, b=5: a * b
assert kw(2) == 10
assert kw(2, b=3) == 6
const = lambda: "c"
assert const() == "c"


def apply(f, v):
    return f(v)


assert apply(lambda s: s + "!", "hi") == "hi!"


def make():
    return lambda n: n * 2


double = make()
assert double(4) == 8
assert (lambda q: q - 1)(5) == 4


def func(k):
    # k is not known inside the lambda (closures are not modeled)
    return lambda x: k + x


y = func(1)
assert y(2) == 3
# TypeError: the lambda takes 1 argument
r = inc(1, 2)
