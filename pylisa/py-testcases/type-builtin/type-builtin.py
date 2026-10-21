import _sv_verifier


class A:
    pass


class B(A):
    pass


# the class of a value is the value of the name of the class
x = 3
assert type(x) is int
assert type(x) is not str
assert type(2.5) is float
assert type("a") is str
assert type(b"a") is bytes
assert type([1]) is list
assert type(A()) is A
assert type(B()) is B
assert type(B()) is not A
assert type(x) is type(4)
assert type(x) is not type("s")
assert type(A()) is type(A())
assert type(x)("5") == 5

# more classes: possible failure
y = "a" if _sv_verifier.nondet_bool() else 4
assert type(y) is str

# classes not modeled (bool, None): unknown
assert type(True) is int
assert type(None) is not int
