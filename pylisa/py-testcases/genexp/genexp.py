xs = [1, 2, 3]

squares = list(x * x for x in xs)
evens = list(x for x in xs if x % 2 == 0)
none = list(x for x in [])
pairs = list(x + y for x in xs for y in xs)

total = sum(x for x in xs)
ok = all(x > 0 for x in xs)
found = any(x == 2 for x in xs)
smallest = min(x for x in xs)

text = ",".join(s for s in ["a", "b"])

# the error is raised while the generator is consumed
zeros = [0, 1]
inverse = list(1 // x for x in zeros)
