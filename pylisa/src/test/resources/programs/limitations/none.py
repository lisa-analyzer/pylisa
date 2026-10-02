class A:
    def __init__(self):
        super().__init__()


class B(A):
    def __init__(self):
        super(B, self).__init__()


def f(x):
    return x


y = f(1)
b = B()
w = [1]
z = w[0]
