def add(a, b, c):
    return a + b + c
xs = [1, 2, 3]
assert add(*xs) == 6
t = (1, 2)
assert add(0, *t) == 3
def total(a, *rest):
    r = a
    for x in rest:
        r = r + x
    return r
assert total(1, *[2, 3]) == 6
assert total(*[5]) == 5

# TypeError: too many arguments
r = add(*[1, 2, 3, 4])
