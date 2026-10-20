a = abs(-3)
assert a == 3
b = abs(2.5)
assert b == 2.5
c = abs(True)
assert c == 1
m1 = min(3, 1, 2)
assert m1 == 1
m2 = max([4, 9, 2])
assert m2 == 9
m3 = min([], default=7)
assert m3 == 7
m4 = max(1, 2.5)
assert m4 == 2.5
m5 = min((5, 4))
assert m5 == 4

# TypeError: str and int cannot be compared
x = min(1, "a")
