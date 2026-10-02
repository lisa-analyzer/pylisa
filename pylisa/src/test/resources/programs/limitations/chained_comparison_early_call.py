def f(a, b, g):
    x = g() < a < b
    return a < g() < b
